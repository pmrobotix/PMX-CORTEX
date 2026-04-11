/*!
 * \file
 * \brief Point d'entrée des tests unitaires common.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"
#include <sys/mman.h>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "MutexTest.hpp"
#include "ThreadTest.hpp"
#include "ChronometerTest.hpp"
#include "LevelTest.hpp"
#include "LoggerTest.hpp"
#include "ExceptionTest.hpp"
#include "PointerListTest.hpp"
#include "FileAppenderTest.hpp"
#include "SvgAppenderTest.hpp"
#include "ActionTimerSchedulerTest.hpp"
#include "RetryPolicyTest.hpp"
#include "TableGeometryTest.hpp"
#include "ObstacleZoneTest.hpp"
#include "DetectionEventTest.hpp"

int main(int argc, char** argv)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall FAILED: " << strerror(errno) << std::endl;
    }
    utils::set_realtime_priority(3, "common-test");

    UnitTestSuite suite;

    suite.addTest(new test::MutexTest());
    suite.addTest(new test::ThreadTest());
    suite.addTest(new test::ChronometerTest());
    suite.addTest(new test::LevelTest());
    suite.addTest(new test::LoggerTest());
    suite.addTest(new test::ExceptionTest());
    suite.addTest(new test::PointerListTest());
    suite.addTest(new test::FileAppenderTest());
    suite.addTest(new test::SvgAppenderTest());
    suite.addTest(new test::ActionTimerSchedulerTest());
    suite.addTest(new test::RetryPolicyTest());
    suite.addTest(new test::TableGeometryTest());
    suite.addTest(new test::ObstacleZoneTest());
    suite.addTest(new test::DetectionEventTest());

    suite.run();

    return 0;
}
