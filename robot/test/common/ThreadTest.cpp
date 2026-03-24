/*!
 * \file
 * \brief Implémentation des tests de la classe utils::Thread.
 */

#include "ThreadTest.hpp"
#include "../../src/common/utils/Chronometer.hpp"

void test::SimpleThread::execute()
{
    for (int i = 0; i < 100; i++) {
        counter_++;
    }
}

void test::MutexCounterThread::execute()
{
    for (int i = 0; i < iterations_; i++) {
        mutex_.lock();
        sharedCounter_++;
        mutex_.unlock();
    }
}

void test::SleepyThread::execute()
{
    executed_ = true;
    // Dort longtemps pour pouvoir etre annule
    this->sleep_for_secs(10);
}

void test::ThreadTest::suite()
{
    testStartAndWait();
    testState();
    testSleepFunctions();
    testNameAndId();
    testConcurrentMutex();
    testCancel();
    testSleepMicros();
}

void test::ThreadTest::testStartAndWait()
{
    SimpleThread t;
    this->assert(t.isFinished(), "thread doit etre CREATED au depart");

    t.start("TestThread", 0);
    t.waitForEnd();

    this->assert(t.counter() == 100, "thread doit avoir execute 100 iterations");
    this->assert(t.isFinished(), "thread doit etre STOPPED apres waitForEnd");
}

void test::ThreadTest::testState()
{
    SimpleThread t;
    this->assert(t.state() == utils::CREATED, "etat initial doit etre CREATED");

    t.start("TestState", 0);
    t.waitForEnd();

    this->assert(t.state() == utils::STOPPED, "etat final doit etre STOPPED");
}

void test::ThreadTest::testSleepFunctions()
{
    utils::Chronometer chrono("testSleep");
    chrono.start();

    utils::sleep_for_millis(50);

    chrono.stop();
    float elapsed = chrono.getElapsedTimeInMilliSec();

    this->assert(elapsed >= 45.0f, "sleep_for_millis(50) doit durer au moins 45ms");
    this->assert(elapsed < 100.0f, "sleep_for_millis(50) ne doit pas depasser 100ms");
}

void test::ThreadTest::testNameAndId()
{
    SimpleThread t;
    t.start("MonThread", 0);
    t.waitForEnd();

    this->assert(t.name() == "MonThread", "Le nom du thread doit etre 'MonThread'");
    this->assert(t.id() != 0, "L'ID du thread doit etre non nul apres start");
}

void test::ThreadTest::testConcurrentMutex()
{
    utils::Mutex mutex;
    int sharedCounter = 0;
    const int iterationsPerThread = 1000;
    const int numThreads = 4;

    MutexCounterThread t1(mutex, sharedCounter, iterationsPerThread);
    MutexCounterThread t2(mutex, sharedCounter, iterationsPerThread);
    MutexCounterThread t3(mutex, sharedCounter, iterationsPerThread);
    MutexCounterThread t4(mutex, sharedCounter, iterationsPerThread);

    t1.start("Counter1", 0);
    t2.start("Counter2", 0);
    t3.start("Counter3", 0);
    t4.start("Counter4", 0);

    t1.waitForEnd();
    t2.waitForEnd();
    t3.waitForEnd();
    t4.waitForEnd();

    int expected = numThreads * iterationsPerThread;
    this->assert(sharedCounter == expected,
        "Le compteur partage doit valoir 4000 avec Mutex");
}

void test::ThreadTest::testCancel()
{
    SleepyThread t;
    t.start("Sleepy", 0);

    // Attendre que le thread soit effectivement lance
    utils::sleep_for_millis(50);
    this->assert(t.wasExecuted(), "Le thread doit avoir demarre son execute()");

    t.cancel();
    t.waitForEnd();

    // Apres cancel + waitForEnd, le thread doit etre termine
    this->assert(true, "cancel() suivi de waitForEnd() ne doit pas bloquer");
}

void test::ThreadTest::testSleepMicros()
{
    utils::Chronometer chrono("testSleepMicros");
    chrono.start();

    utils::sleep_for_micros(5000); // 5ms

    chrono.stop();
    float elapsed = chrono.getElapsedTimeInMilliSec();

    this->assert(elapsed >= 4.0f, "sleep_for_micros(5000) doit durer au moins 4ms");
    this->assert(elapsed < 50.0f, "sleep_for_micros(5000) ne doit pas depasser 50ms");
}
