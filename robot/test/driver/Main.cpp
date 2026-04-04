/*!
 * \file
 * \brief Point d'entree des tests unitaires driver.
 *
 * Teste les drivers via leurs interfaces abstraites.
 * En SIMU, les implementations simulation sont utilisees.
 * En ARM, les implementations hardware sont utilisees.
 */

#include "../../src/common/thread/Thread.hpp"
#include "../suite/UnitTestSuite.hpp"
#include <sys/mman.h>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "LedDriverTest.hpp"
#include "SwitchDriverTest.hpp"
#ifdef SIMU
#include "NavigatorTest.hpp"
#endif

int main(int, char**)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall FAILED: " << strerror(errno) << std::endl;
    }
    utils::set_realtime_priority(3, "driver-test");

    UnitTestSuite suite;

    suite.addTest(new test::LedDriverTest());
    suite.addTest(new test::SwitchDriverTest());
#ifdef SIMU
    suite.addTest(new test::NavigatorTest());
#endif

    suite.run();

    return 0;
}
