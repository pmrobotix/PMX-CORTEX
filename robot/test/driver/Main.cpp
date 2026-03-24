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

#include "LedDriverTest.hpp"
#include "SwitchDriverTest.hpp"

int main(int, char**)
{
    utils::set_realtime_priority(3, "driver-test");

    UnitTestSuite suite;

    suite.addTest(new test::LedDriverTest());
    suite.addTest(new test::SwitchDriverTest());

    suite.run();

    return 0;
}
