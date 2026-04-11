/*!
 * \file
 * \brief Benchmark de precision du scheduler actuel (ActionTimerScheduler).
 *
 * Mesure le jitter et la precision des intervalles en microsecondes pour
 * la methode actuelle (1 thread + clock_nanosleep ABSTIME).
 * A comparer avec PosixTimerBench (legacy SIGEV_THREAD/SIGALRM) sur la meme
 * machine, et entre x86 (SIMU) et ARM (OPOS6UL).
 */

#ifndef TEST_ACTIONTIMERSCHEDULERBENCH_HPP
#define TEST_ACTIONTIMERSCHEDULERBENCH_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class ActionTimerSchedulerBench: public UnitTest {
private:

    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("test::ActionTimerSchedulerBench");
        return instance;
    }

public:

    ActionTimerSchedulerBench() :
            UnitTest("ActionTimerSchedulerBench - precision scheduler actuel")
    {
    }

    virtual ~ActionTimerSchedulerBench() {
    }

    virtual void suite();

    void benchTimer100ms();
    void benchTimer10ms();
    void benchTimer1ms();
    void benchMultipleTimersConcurrent();
};
}

#endif
