/*!
 * \file
 * \brief Bench du timer POSIX standalone.
 */

#include "TimerPosixStandaloneBench.hpp"
#include "TimerPosix.hpp"

#include <unistd.h>

#include "utils/Chronometer.hpp"
#include "thread/Thread.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

using namespace utils;

void test::TimerPosixStandaloneBench::suite()
{
    this->test();
}

void test::TimerPosixStandaloneBench::test()
{
    logger().info() << "TimerPosixStandaloneBench::test()..." << logs::end;

    utils::Chronometer chrono("TimerPosixStandaloneBench");
    chrono.start();

    logger().info() << "create timers " << chrono.getElapsedTimeInMicroSec() << logs::end;
    TimerPosix *timer1 = new TimerPosix("timer1");
    TimerPosix *timer2 = new TimerPosix("timer2");
    TimerPosix *timer3 = new TimerPosix("timer3");

    sleep(1);
    logger().info() << "start timers " << chrono.getElapsedTimeInMicroSec() << logs::end;
    timer1->start();
    utils::sleep_for_secs(3);

    timer2->start();
    utils::sleep_for_millis(200);
    timer3->start();
    logger().info() << "wait timers... " << chrono.getElapsedTimeInMicroSec() << logs::end;

    while (true) {
        utils::sleep_for_micros(200000);
        logger().info() << " - " << chrono.getElapsedTimeInMicroSec() << logs::end;
        if (chrono.getElapsedTimeInSec() > 10)
            break;
    }

    delete timer1;
    delete timer2;
    delete timer3;

    logger().info() << "TimerPosixStandaloneBench::test() END" << logs::end;
}
