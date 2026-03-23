/*!
 * \file
 * \brief Point d'entrée des tests unitaires common.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"

#include "MutexTest.hpp"
#include "ThreadTest.hpp"
#include "ChronometerTest.hpp"

int main(int argc, char** argv)
{
    utils::set_realtime_priority(3, "common-test");

    UnitTestSuite suite;

    suite.addTest(new test::MutexTest());
    suite.addTest(new test::ThreadTest());
    suite.addTest(new test::ChronometerTest());

    suite.run();

    return 0;
}
