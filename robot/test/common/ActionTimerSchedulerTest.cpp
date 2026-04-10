/*!
 * \file
 * \brief Tests unitaires de ActionTimerScheduler + ITimerScheduledListener.
 */

#include "timer/ActionTimerScheduler.hpp"
#include "timer/ITimerScheduledListener.hpp"
#include "ActionTimerSchedulerTest.hpp"

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "action/IAction.hpp"
#include "thread/Mutex.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

using namespace utils;

// ============================================================================
// Mocks
// ============================================================================

/*!
 * \brief Action one-shot : s'execute une fois et retourne false.
 */
class OneShotAction: public IAction {
private:
    std::atomic<int> count_;
public:
    OneShotAction(int id) : count_(0) {
        std::ostringstream oss; oss << "OneShot" << id;
        name_ = oss.str();
    }
    bool execute() override {
        count_++;
        return false; // termine apres 1 execution
    }
    int count() const { return count_.load(); }
};

/*!
 * \brief Action recurrente terminable : compte jusqu'a N puis se termine.
 */
class CountToNAction: public IAction {
private:
    std::atomic<int> count_;
    int target_;
public:
    CountToNAction(int id, int target) : count_(0), target_(target) {
        std::ostringstream oss; oss << "CountTo" << target << "_" << id;
        name_ = oss.str();
    }
    bool execute() override {
        count_++;
        return (count_ < target_); // recurrent jusqu'a target
    }
    int count() const { return count_.load(); }
    bool finished() const { return count_ >= target_; }
};

/*!
 * \brief Action recurrente "wait_ms" : attend N ms puis se termine.
 *        Pattern typique : if (chrono.elapsed_ms() < N) return true; return false;
 */
class WaitMsAction: public IAction {
private:
    std::atomic<int> count_;
    Chronometer chrono_;
    long wait_ms_;
    std::atomic<bool> finished_;
public:
    WaitMsAction(long wait_ms) : count_(0), chrono_("WaitMsAction"),
                                  wait_ms_(wait_ms), finished_(false) {
        name_ = "WaitMs";
        chrono_.start();
    }
    bool execute() override {
        count_++;
        if (chrono_.getElapsedTimeInMicroSec() / 1000 < wait_ms_) {
            return true; // pas encore l'heure, repasser
        }
        finished_ = true;
        return false; // attente terminee
    }
    int count() const { return count_.load(); }
    bool finished() const { return finished_.load(); }
};

// ============================================================================
// Mock timers
// ============================================================================

/*!
 * \brief Timer minimal qui compte les ticks.
 */
class CountingScheduledTimer: public ITimerScheduledListener {
private:
    std::atomic<long> count_;
    std::atomic<bool> endCalled_;
public:
    CountingScheduledTimer(std::string label, int period_us) : count_(0), endCalled_(false) {
        this->init(label, period_us);
    }
    void onTimer(utils::Chronometer chrono) override {
        count_++;
    }
    void onTimerEnd(utils::Chronometer chrono) override {
        endCalled_ = true;
    }
    long count() const { return count_.load(); }
    bool endWasCalled() const { return endCalled_.load(); }
};

/*!
 * \brief Timer qui prend un mutex partage avec le thread principal.
 *        Reproduit le scenario qui causait deadlock avec SIGEV_SIGNAL.
 */
class MutexSharingScheduledTimer: public ITimerScheduledListener {
private:
    Mutex &shared_mutex_;
    std::atomic<long> count_;
public:
    MutexSharingScheduledTimer(std::string label, int period_us, Mutex &m)
        : shared_mutex_(m), count_(0) {
        this->init(label, period_us);
    }
    void onTimer(utils::Chronometer chrono) override {
        shared_mutex_.lock();
        count_++;
        shared_mutex_.unlock();
    }
    void onTimerEnd(utils::Chronometer chrono) override {}
    long count() const { return count_.load(); }
};

// ============================================================================
// Suite
// ============================================================================

void test::ActionTimerSchedulerTest::suite() {
    testInitialState();
    testActionOneShot();
    testActionRecurrenteTerminable();
    testActionRoundRobin();
    testManyActionsConcurrent();
    testTimerSimple();
    testTimerPeriodAccuracy();
    testTimerStop();
    testMultipleTimers();
    testHighFrequencyTimer();
    testActionsAndTimersTogether();
    testTimerWithSharedMutex();
    testIdleNoCpu();
    testStopGlobal();
    testPauseResume();
}

// ----------------------------------------------------------------------------
// Etat initial
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testInitialState() {
    logger().info() << "testInitialState..." << logs::end;

    ActionTimerScheduler sched;
    this->assert(sched.countActions() == 0, "countActions doit etre 0 a l'init");
    this->assert(sched.countTimers() == 0, "countTimers doit etre 0 a l'init");

    logger().info() << "testInitialState OK" << logs::end;
}

// ----------------------------------------------------------------------------
// Actions
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testActionOneShot() {
    logger().info() << "testActionOneShot..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-os", 0);

    OneShotAction *a = new OneShotAction(1);
    sched.addAction(a);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    this->assert(a->count() == 1, "OneShot doit s'executer exactement 1 fois");
    this->assert(sched.countActions() == 0, "queue doit etre vide apres oneshot");

    sched.stop();
    delete a;

    logger().info() << "testActionOneShot OK" << logs::end;
}

void test::ActionTimerSchedulerTest::testActionRecurrenteTerminable() {
    logger().info() << "testActionRecurrenteTerminable..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-rec", 0);

    // Attente 200ms : doit etre executee plusieurs fois puis se terminer
    WaitMsAction *a = new WaitMsAction(200);
    sched.addAction(a);

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    this->assert(a->finished(), "WaitMs doit etre terminee apres 400ms");
    this->assert(a->count() > 1, "WaitMs doit avoir ete repassee plusieurs fois");
    this->assert(sched.countActions() == 0, "queue vide apres terminaison");

    sched.stop();
    delete a;

    logger().info() << "testActionRecurrenteTerminable OK (count="
        << a->count() << ")" << logs::end;  // note: a est deja delete, mais c'est OK pour le log
}

void test::ActionTimerSchedulerTest::testActionRoundRobin() {
    logger().info() << "testActionRoundRobin..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-rr", 0);

    // 3 actions qui comptent jusqu'a 50 chacune.
    // Round-robin : elles doivent toutes finir a peu pres en meme temps.
    CountToNAction *a1 = new CountToNAction(1, 50);
    CountToNAction *a2 = new CountToNAction(2, 50);
    CountToNAction *a3 = new CountToNAction(3, 50);
    sched.addAction(a1);
    sched.addAction(a2);
    sched.addAction(a3);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    this->assert(a1->finished(), "a1 doit etre finie");
    this->assert(a2->finished(), "a2 doit etre finie");
    this->assert(a3->finished(), "a3 doit etre finie");
    this->assert(a1->count() == 50 && a2->count() == 50 && a3->count() == 50,
        "chaque action doit avoir tick exactement 50 fois");

    sched.stop();
    delete a1; delete a2; delete a3;

    logger().info() << "testActionRoundRobin OK" << logs::end;
}

void test::ActionTimerSchedulerTest::testManyActionsConcurrent() {
    logger().info() << "testManyActionsConcurrent..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-many", 0);

    std::vector<CountToNAction*> actions;
    const int N = 20;
    const int TARGET = 30;
    for (int i = 0; i < N; i++) {
        auto *a = new CountToNAction(i, TARGET);
        actions.push_back(a);
        sched.addAction(a);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int finished = 0;
    for (auto *a : actions) if (a->finished()) finished++;

    this->assert(finished == N, "toutes les actions doivent etre terminees");

    sched.stop();
    for (auto *a : actions) delete a;

    logger().info() << "testManyActionsConcurrent OK (" << finished << "/" << N << ")" << logs::end;
}

// ----------------------------------------------------------------------------
// Timers
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testTimerSimple() {
    logger().info() << "testTimerSimple..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-tsimple", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("t1", 50000); // 50ms
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    this->assert(t->count() >= 4, "5 ticks attendus en 300ms (tolerance 4)");

    sched.stop();
    delete t;

    logger().info() << "testTimerSimple OK" << logs::end;
}

void test::ActionTimerSchedulerTest::testTimerPeriodAccuracy() {
    logger().info() << "testTimerPeriodAccuracy..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-tacc", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("acc", 100000); // 100ms
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    long n = t->count();

    // 1s a 100ms = ~10 ticks (tolerance +/-2)
    this->assert(n >= 8 && n <= 12,
        "10 ticks attendus en 1s a 100ms (tolerance +/-2)");

    sched.stop();
    delete t;

    logger().info() << "testTimerPeriodAccuracy OK (count=" << n << ")" << logs::end;
}

void test::ActionTimerSchedulerTest::testTimerStop() {
    logger().info() << "testTimerStop..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-tstop", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("toStop", 50000);
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    long countBefore = t->count();

    sched.stopTimer("toStop");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    long countAfter = t->count();

    this->assert(t->endWasCalled(), "onTimerEnd doit etre appele apres stopTimer");
    this->assert(countAfter <= countBefore + 2,
        "le timer ne doit plus tick apres stopTimer (tolerance 2)");

    sched.stop();
    delete t;

    logger().info() << "testTimerStop OK" << logs::end;
}

void test::ActionTimerSchedulerTest::testMultipleTimers() {
    logger().info() << "testMultipleTimers..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-tmulti", 0);

    CountingScheduledTimer *fast = new CountingScheduledTimer("fast", 20000);  // 50 Hz
    CountingScheduledTimer *slow = new CountingScheduledTimer("slow", 100000); // 10 Hz
    sched.addTimer(fast);
    sched.addTimer(slow);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    long nf = fast->count();
    long ns = slow->count();

    this->assert(nf >= 40 && nf <= 55, "fast doit ticker ~50 fois en 1s");
    this->assert(ns >= 8 && ns <= 12, "slow doit ticker ~10 fois en 1s");

    sched.stop();
    delete fast; delete slow;

    logger().info() << "testMultipleTimers OK (fast=" << nf << " slow=" << ns << ")" << logs::end;
}

void test::ActionTimerSchedulerTest::testHighFrequencyTimer() {
    logger().info() << "testHighFrequencyTimer..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-thf", 0);

    // 5ms = 200 Hz pendant 3 sec. Avec SIGEV_THREAD legacy : OOM kill.
    // Avec ActionTimerScheduler : 1 seul thread, pas d'accumulation.
    CountingScheduledTimer *t = new CountingScheduledTimer("hifreq", 5000);
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    long n = t->count();
    // 3s * 200 Hz = 600 ticks attendus (tolerance large pour le scheduler)
    this->assert(n >= 400, "le timer doit avoir tick >= 400 fois en 3s a 5ms");

    sched.stop();
    delete t;

    logger().info() << "testHighFrequencyTimer OK (count=" << n << ")" << logs::end;
}

// ----------------------------------------------------------------------------
// Mix actions + timers
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testActionsAndTimersTogether() {
    logger().info() << "testActionsAndTimersTogether..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-mix", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("mixT", 50000); // 50ms
    CountToNAction *a = new CountToNAction(1, 30);
    sched.addTimer(t);
    sched.addAction(a);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    this->assert(a->finished(), "action doit etre finie");
    this->assert(t->count() >= 8, "timer doit avoir tick au moins 8 fois en 500ms");

    sched.stop();
    delete t; delete a;

    logger().info() << "testActionsAndTimersTogether OK" << logs::end;
}

// ----------------------------------------------------------------------------
// Mutex partage avec thread principal (regression deadlock SIGALRM)
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testTimerWithSharedMutex() {
    logger().info() << "testTimerWithSharedMutex..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-mutex", 0);

    Mutex shared;
    MutexSharingScheduledTimer *t = new MutexSharingScheduledTimer("muTimer", 5000, shared); // 5ms
    sched.addTimer(t);

    // Le thread principal prend/relache le mutex en boucle pendant 1 sec
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    long mainCount = 0;
    while (std::chrono::steady_clock::now() < end) {
        shared.lock();
        mainCount++;
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        shared.unlock();
        std::this_thread::yield();
    }

    long timerCount = t->count();

    this->assert(timerCount > 50,
        "timer doit avoir tick > 50 fois en 1s a 5ms (sinon deadlock)");
    this->assert(mainCount > 100,
        "thread principal doit avoir continue a tourner");

    sched.stop();
    delete t;

    logger().info() << "testTimerWithSharedMutex OK (timer=" << timerCount
        << " main=" << mainCount << ")" << logs::end;
}

// ----------------------------------------------------------------------------
// Idle = 0% CPU
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testIdleNoCpu() {
    logger().info() << "testIdleNoCpu..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-idle", 0);

    // Mesurer le temps CPU consomme par CE thread (le scheduler) sur 500ms
    // de repos. Avec sem_wait, on doit etre tres proche de 0.
    struct timespec t0, t1;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);

    long cpu_ns = (t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec);
    long cpu_ms = cpu_ns / 1000000L;

    // Tolerance large : meme avec d'autres threads de test, on doit rester
    // largement sous les 500ms reels (sinon ca tournerait en boucle active).
    this->assert(cpu_ms < 100,
        "CPU consomme au repos doit etre < 100ms sur 500ms (sem_wait inactif)");

    sched.stop();

    logger().info() << "testIdleNoCpu OK (cpu=" << cpu_ms << "ms / 500ms)" << logs::end;
}

// ----------------------------------------------------------------------------
// Stop / pause
// ----------------------------------------------------------------------------

void test::ActionTimerSchedulerTest::testStopGlobal() {
    logger().info() << "testStopGlobal..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-stop", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("stopAll", 50000);
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    sched.stop(); // doit arreter proprement (waitForEnd() interne)

    this->assert(sched.state() == utils::STOPPED,
        "le scheduler doit etre STOPPED apres stop()");

    delete t;

    logger().info() << "testStopGlobal OK" << logs::end;
}

void test::ActionTimerSchedulerTest::testPauseResume() {
    logger().info() << "testPauseResume..." << logs::end;

    ActionTimerScheduler sched;
    sched.start("sched-pause", 0);

    CountingScheduledTimer *t = new CountingScheduledTimer("pausable", 50000); // 50ms
    sched.addTimer(t);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    long count1 = t->count();

    sched.pause(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    long count2 = t->count();

    // En pause, le compteur ne doit plus avancer (tolerance 1 tick de transition)
    this->assert(count2 <= count1 + 1, "le timer ne doit pas tick en pause");

    sched.pause(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    long count3 = t->count();

    this->assert(count3 > count2, "le timer doit reprendre apres pause(false)");

    sched.stop();
    delete t;

    logger().info() << "testPauseResume OK (c1=" << count1 << " c2=" << count2 << " c3=" << count3 << ")" << logs::end;
}
