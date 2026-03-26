/*!
 * \file
 * \brief Tests unitaires de la classe ActionManagerTimer.
 */

#include "action/ActionManagerTimer.hpp"
#include "ActionManagerTimerTest.hpp"

#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "action/IAction.hpp"
#include "action/ITimerListener.hpp"
#include "action/ITimerPosixListener.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

using namespace utils;

// --- Mocks ---

class CountingAction: public IAction {
private:
    std::atomic<long> count_;
    long limit_;

public:
    CountingAction(int id, long limit) : count_(0), limit_(limit)
    {
        std::ostringstream oss;
        oss << "CountingAction" << id;
        name_ = oss.str();
    }

    virtual ~CountingAction() {}

    bool execute() {
        count_++;
        return (count_ < limit_);
    }

    long count() const { return count_.load(); }
    void reset() { count_ = 0; }
};

class OrderingAction: public IAction {
private:
    int id_;
    std::vector<int> &order_;

public:
    OrderingAction(int id, std::vector<int> &order) : id_(id), order_(order)
    {
        std::ostringstream oss;
        oss << "OrderingAction" << id;
        name_ = oss.str();
    }

    virtual ~OrderingAction() {}

    bool execute() {
        order_.push_back(id_);
        return false; // une seule execution
    }
};

class CountingPosixTimer: public ITimerPosixListener {
private:
    std::atomic<long> count_;
    std::atomic<bool> endCalled_;

public:
    CountingPosixTimer(std::string label, int time_ms) : count_(0), endCalled_(false)
    {
        this->init(label, time_ms * 1000);
    }

    virtual ~CountingPosixTimer() {}

    void onTimer(utils::Chronometer chrono) {
        count_++;
    }

    void onTimerEnd(utils::Chronometer chrono) {
        endCalled_ = true;
    }

    long count() const { return count_.load(); }
    bool endWasCalled() const { return endCalled_.load(); }
};

class CountingTimer: public ITimerListener {
private:
    std::atomic<bool> endCalled_;

public:
    CountingTimer(std::string label, int timeSpan_ms) : endCalled_(false)
    {
        timeSpan_ms_ = timeSpan_ms;
        name_ = label;
    }

    virtual ~CountingTimer() {}

    void onTimer(utils::Chronometer chrono) {}
    void onTimerEnd(utils::Chronometer chrono) {
        endCalled_ = true;
    }

    bool endWasCalled() const { return endCalled_.load(); }
};

// --- Suite ---

void test::ActionManagerTimerTest::suite() {
    testInitialState();
    testAddRemoveActions();
    testAddRemoveTimers();
    testAddRemovePosixTimers();
    testActionExecution();
    testStopRestart();
    testPauseResume();
    testActionFinishesAndRemoved();
    testMultipleActionsOrdering();
    testStopTimerByName();
    testStopPTimerByName();
    testStopAllPTimers();
    testConcurrentAddAction();
}

// --- Tests existants ---

void test::ActionManagerTimerTest::testInitialState() {
    logger().info() << "testInitialState..." << logs::end;

    ActionManagerTimer manager;
    this->assert(manager.countActions() == 0, "countActions() doit etre 0 a l'init");
    this->assert(manager.countTimers() == 0, "countTimers() doit etre 0 a l'init");
    this->assert(manager.countPTimers() == 0, "countPTimers() doit etre 0 a l'init");

    logger().info() << "testInitialState OK" << logs::end;
}

void test::ActionManagerTimerTest::testAddRemoveActions() {
    logger().info() << "testAddRemoveActions..." << logs::end;

    ActionManagerTimer manager;

    CountingAction *a1 = new CountingAction(1, 100);
    CountingAction *a2 = new CountingAction(2, 100);

    manager.addAction(a1);
    this->assert(manager.countActions() == 1, "countActions() doit etre 1 apres 1 ajout");

    manager.addAction(a2);
    this->assert(manager.countActions() == 2, "countActions() doit etre 2 apres 2 ajouts");

    manager.clearActions();
    this->assert(manager.countActions() == 0, "countActions() doit etre 0 apres clearActions()");

    delete a1;
    delete a2;

    logger().info() << "testAddRemoveActions OK" << logs::end;
}

void test::ActionManagerTimerTest::testAddRemoveTimers() {
    logger().info() << "testAddRemoveTimers..." << logs::end;

    ActionManagerTimer manager;

    CountingTimer *t1 = new CountingTimer("timer1", 100);
    CountingTimer *t2 = new CountingTimer("timer2", 200);

    manager.addTimer(t1);
    this->assert(manager.countTimers() == 1, "countTimers() doit etre 1 apres 1 ajout");

    manager.addTimer(t2);
    this->assert(manager.countTimers() == 2, "countTimers() doit etre 2 apres 2 ajouts");

    manager.clearTimers();
    this->assert(manager.countTimers() == 0, "countTimers() doit etre 0 apres clearTimers()");

    delete t1;
    delete t2;

    logger().info() << "testAddRemoveTimers OK" << logs::end;
}

void test::ActionManagerTimerTest::testAddRemovePosixTimers() {
    logger().info() << "testAddRemovePosixTimers..." << logs::end;

    ActionManagerTimer manager;

    CountingPosixTimer *pt1 = new CountingPosixTimer("ptimer1", 100);

    manager.addTimer(pt1);
    manager.start("testPosixTimers", 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    this->assert(manager.countPTimers() >= 1, "countPTimers() doit etre >= 1 apres demarrage");
    this->assert(manager.findPTimer("ptimer1"), "findPTimer doit trouver ptimer1");

    manager.stopAllPTimers();
    this->assert(manager.countPTimers() == 0, "countPTimers() doit etre 0 apres stopAllPTimers()");

    manager.stop();
    delete pt1;

    logger().info() << "testAddRemovePosixTimers OK" << logs::end;
}

void test::ActionManagerTimerTest::testActionExecution() {
    logger().info() << "testActionExecution..." << logs::end;

    ActionManagerTimer manager;

    CountingAction *a1 = new CountingAction(1, 5);

    manager.addAction(a1);
    manager.start("testActionExecution", 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    this->assert(a1->count() >= 5, "l'action doit avoir ete executee au moins 5 fois");
    this->assert(manager.countActions() == 0, "countActions() doit etre 0 apres fin d'action (retour false)");

    manager.stop();
    delete a1;

    logger().info() << "testActionExecution OK" << logs::end;
}

// --- Nouveaux tests ---

void test::ActionManagerTimerTest::testStopRestart() {
    logger().info() << "testStopRestart..." << logs::end;

    ActionManagerTimer manager;

    // premier cycle
    CountingAction *a1 = new CountingAction(1, 3);
    manager.addAction(a1);
    manager.start("cycle1", 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    manager.stop();

    this->assert(a1->count() >= 3, "cycle1: l'action doit avoir termine");

    // deuxieme cycle — verifie que stop() a bien reinitialise le flag
    a1->reset();
    manager.addAction(a1);
    manager.start("cycle2", 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    this->assert(a1->count() >= 3, "cycle2: l'action doit avoir termine apres restart");

    manager.stop();
    delete a1;

    logger().info() << "testStopRestart OK" << logs::end;
}

void test::ActionManagerTimerTest::testPauseResume() {
    logger().info() << "testPauseResume..." << logs::end;

    ActionManagerTimer manager;
    manager.start("testPause", 2);

    // laisser le manager demarrer
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // mettre en pause AVANT d'ajouter une action
    manager.pause(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ajouter une action pendant la pause
    CountingAction *a1 = new CountingAction(1, 1);
    manager.addAction(a1);

    // attendre — l'action ne doit PAS s'executer
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    this->assert(a1->count() == 0, "l'action ne doit pas s'executer pendant la pause");
    this->assert(manager.countActions() == 1, "l'action doit rester dans la queue");

    // resume — l'action doit s'executer
    manager.pause(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    this->assert(a1->count() == 1, "l'action doit s'executer apres resume");

    manager.stop();
    delete a1;

    logger().info() << "testPauseResume OK" << logs::end;
}

void test::ActionManagerTimerTest::testActionFinishesAndRemoved() {
    logger().info() << "testActionFinishesAndRemoved..." << logs::end;

    ActionManagerTimer manager;

    // action qui s'execute 1 seule fois (limit=1, retourne false)
    CountingAction *a1 = new CountingAction(1, 1);
    manager.addAction(a1);
    manager.start("testFinish", 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    this->assert(a1->count() == 1, "l'action doit avoir ete executee exactement 1 fois");
    this->assert(manager.countActions() == 0, "l'action doit etre retiree apres retour false");

    manager.stop();
    delete a1;

    logger().info() << "testActionFinishesAndRemoved OK" << logs::end;
}

void test::ActionManagerTimerTest::testMultipleActionsOrdering() {
    logger().info() << "testMultipleActionsOrdering..." << logs::end;

    ActionManagerTimer manager;
    std::vector<int> order;

    OrderingAction *a1 = new OrderingAction(1, order);
    OrderingAction *a2 = new OrderingAction(2, order);
    OrderingAction *a3 = new OrderingAction(3, order);

    manager.addAction(a1);
    manager.addAction(a2);
    manager.addAction(a3);
    manager.start("testOrder", 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    this->assert(order.size() == 3, "les 3 actions doivent avoir ete executees");
    if (order.size() == 3) {
        this->assert(order[0] == 1, "action 1 doit etre executee en premier (FIFO)");
        this->assert(order[1] == 2, "action 2 doit etre executee en deuxieme");
        this->assert(order[2] == 3, "action 3 doit etre executee en troisieme");
    }

    manager.stop();
    delete a1;
    delete a2;
    delete a3;

    logger().info() << "testMultipleActionsOrdering OK" << logs::end;
}

void test::ActionManagerTimerTest::testStopTimerByName() {
    logger().info() << "testStopTimerByName..." << logs::end;

    ActionManagerTimer manager;

    CountingTimer *t1 = new CountingTimer("timerA", 100);
    CountingTimer *t2 = new CountingTimer("timerB", 200);

    manager.addTimer(t1);
    manager.addTimer(t2);
    this->assert(manager.countTimers() == 2, "2 timers ajoutes");

    manager.stopTimer("timerA");
    this->assert(t1->endWasCalled(), "onTimerEnd doit etre appele sur timerA");
    this->assert(manager.countTimers() == 1, "1 timer restant apres stopTimer(timerA)");

    manager.stopTimer("timerB");
    this->assert(t2->endWasCalled(), "onTimerEnd doit etre appele sur timerB");
    this->assert(manager.countTimers() == 0, "0 timer restant");

    delete t1;
    delete t2;

    logger().info() << "testStopTimerByName OK" << logs::end;
}

void test::ActionManagerTimerTest::testStopPTimerByName() {
    logger().info() << "testStopPTimerByName..." << logs::end;

    ActionManagerTimer manager;

    CountingPosixTimer *pt1 = new CountingPosixTimer("ptimerA", 100);
    CountingPosixTimer *pt2 = new CountingPosixTimer("ptimerB", 100);

    manager.addTimer(pt1);
    manager.addTimer(pt2);
    manager.start("testStopPTimer", 2);

    // attendre que les ptimers soient transferes et demarres
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    this->assert(manager.countPTimers() == 2, "2 ptimers actifs");

    manager.stopPTimer("ptimerA");
    this->assert(pt1->endWasCalled(), "onTimerEnd doit etre appele sur ptimerA");
    this->assert(manager.countPTimers() == 1, "1 ptimer restant");

    manager.stopPTimer("ptimerB");
    this->assert(pt2->endWasCalled(), "onTimerEnd doit etre appele sur ptimerB");
    this->assert(manager.countPTimers() == 0, "0 ptimer restant");

    manager.stop();
    delete pt1;
    delete pt2;

    logger().info() << "testStopPTimerByName OK" << logs::end;
}

void test::ActionManagerTimerTest::testStopAllPTimers() {
    logger().info() << "testStopAllPTimers..." << logs::end;

    ActionManagerTimer manager;

    CountingPosixTimer *pt1 = new CountingPosixTimer("pt1", 100);
    CountingPosixTimer *pt2 = new CountingPosixTimer("pt2", 100);
    CountingPosixTimer *pt3 = new CountingPosixTimer("pt3", 100);
    CountingTimer *t1 = new CountingTimer("t1", 100);

    manager.addTimer(pt1);
    manager.addTimer(pt2);
    manager.addTimer(pt3);
    manager.addTimer(t1);
    manager.start("testStopAll", 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    manager.stopAllPTimers();

    this->assert(manager.countPTimers() == 0, "0 ptimers apres stopAllPTimers");
    this->assert(manager.countTimers() == 0, "0 timers apres stopAllPTimers");
    this->assert(pt1->endWasCalled(), "onTimerEnd appele sur pt1");
    this->assert(pt2->endWasCalled(), "onTimerEnd appele sur pt2");
    this->assert(pt3->endWasCalled(), "onTimerEnd appele sur pt3");
    this->assert(t1->endWasCalled(), "onTimerEnd appele sur t1");

    manager.stop();
    delete pt1;
    delete pt2;
    delete pt3;
    delete t1;

    logger().info() << "testStopAllPTimers OK" << logs::end;
}

void test::ActionManagerTimerTest::testConcurrentAddAction() {
    logger().info() << "testConcurrentAddAction..." << logs::end;

    ActionManagerTimer manager;
    manager.start("testConcurrent", 2);

    const int NB_PER_THREAD = 50;
    std::atomic<int> totalExecuted(0);

    // action qui incremente un compteur global
    class AtomicAction: public IAction {
    private:
        std::atomic<int> &counter_;
    public:
        AtomicAction(std::atomic<int> &counter) : counter_(counter) {
            name_ = "AtomicAction";
        }
        virtual ~AtomicAction() {}
        bool execute() {
            counter_++;
            return false; // une seule execution
        }
    };

    // deux threads ajoutent des actions en parallele
    std::vector<AtomicAction*> actions;
    for (int i = 0; i < NB_PER_THREAD * 2; i++) {
        actions.push_back(new AtomicAction(totalExecuted));
    }

    std::thread t1([&]() {
        for (int i = 0; i < NB_PER_THREAD; i++) {
            manager.addAction(actions[i]);
            std::this_thread::yield();
        }
    });

    std::thread t2([&]() {
        for (int i = NB_PER_THREAD; i < NB_PER_THREAD * 2; i++) {
            manager.addAction(actions[i]);
            std::this_thread::yield();
        }
    });

    t1.join();
    t2.join();

    // attente que toutes les actions soient executees
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    this->assert(totalExecuted.load() == NB_PER_THREAD * 2,
            "toutes les actions doivent avoir ete executees sans crash");
    this->assert(manager.countActions() == 0,
            "toutes les actions doivent avoir ete retirees");

    manager.stop();

    for (auto *a : actions)
        delete a;

    logger().info() << "testConcurrentAddAction OK" << logs::end;
}
