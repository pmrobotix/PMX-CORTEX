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

void test::ThreadTest::suite()
{
    testStartAndWait();
    testState();
    testSleepFunctions();
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
