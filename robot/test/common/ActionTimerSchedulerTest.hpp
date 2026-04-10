/*!
 * \file
 * \brief Tests unitaires de ActionTimerScheduler + ITimerScheduledListener.
 *
 * Valide le scheduler unique (1 thread pour toutes les IAction et tous les
 * ITimerScheduledListener), nouvelle architecture remplacant ActionManagerTimer
 * (legacy SIGEV_THREAD) et ITimerThreadListener (1 thread par timer).
 */

#ifndef TEST_ACTIONTIMERSCHEDULERTEST_HPP
#define TEST_ACTIONTIMERSCHEDULERTEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionTimerSchedulerTest: public UnitTest {
private:

    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionTimerSchedulerTest");
        return instance;
    }

public:

    ActionTimerSchedulerTest() :
            UnitTest("ActionTimerSchedulerTest")
    {
    }

    virtual ~ActionTimerSchedulerTest() {
    }

    virtual void suite();

    // --- Etat initial ---
    void testInitialState();

    // --- Actions ---
    void testActionOneShot();
    void testActionRecurrenteTerminable();    // pattern: condition de fin
    void testActionRoundRobin();              // 3 actions recurrentes -> equite
    void testManyActionsConcurrent();         // stress

    // --- Timers ---
    void testTimerSimple();
    void testTimerPeriodAccuracy();           // precision du timing
    void testTimerStop();
    void testMultipleTimers();
    void testHighFrequencyTimer();            // 5ms = 200 Hz, pas d'OOM

    // --- Mix actions + timers ---
    void testActionsAndTimersTogether();

    // --- Critique : mutex partage avec thread principal ---
    void testTimerWithSharedMutex();          // verif que pas de deadlock

    // --- CPU au repos ---
    void testIdleNoCpu();                     // sem_wait : 0% CPU au repos

    // --- Stop / pause ---
    void testStopGlobal();
    void testPauseResume();
};
}

#endif
