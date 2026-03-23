/*!
 * \file
 * \brief Implémentation des tests de la classe utils::Chronometer.
 */

#include "ChronometerTest.hpp"
#include "../../src/common/utils/Chronometer.hpp"
#include <thread>
#include <chrono>

void test::ChronometerTest::suite()
{
    testStartStop();
    testElapsedTime();
    testSetTimerAndWait();
}

void test::ChronometerTest::testStartStop()
{
    utils::Chronometer c("testStartStop");

    this->assert(!c.started(), "chronometer ne doit pas etre demarre apres construction");

    c.start();
    this->assert(c.started(), "chronometer doit etre demarre apres start()");

    c.stop();
    this->assert(!c.started(), "chronometer ne doit pas etre demarre apres stop()");
}

void test::ChronometerTest::testElapsedTime()
{
    utils::Chronometer c("testElapsed");
    c.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    float elapsed = c.getElapsedTimeInMilliSec();
    this->assert(elapsed >= 40.0f, "elapsed doit etre >= 40ms apres sleep(50ms)");
    this->assert(elapsed < 100.0f, "elapsed doit etre < 100ms apres sleep(50ms)");

    c.stop();
    float stopped = c.getElapsedTimeInMilliSec();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    float afterStop = c.getElapsedTimeInMilliSec();
    this->assert(afterStop == stopped, "elapsed ne doit plus bouger apres stop()");
}

void test::ChronometerTest::testSetTimerAndWait()
{
    utils::Chronometer c("testTimer");
    c.setTimer(10000); // 10ms period

    int count = c.waitTimer();
    this->assert(count == 1, "waitTimer doit retourner le numero de periode (1)");

    count = c.waitTimer();
    this->assert(count == 2, "waitTimer doit retourner 2 au 2e appel");
}
