/*!
 * \file
 * \brief Test manuel du driver d'asservissement.
 */

#include "AsservDriverManualTest.hpp"

#include <unistd.h>

#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"

test::AsservDriverManualTest::AsservDriverManualTest() :
        UnitTest("AsservDriverManualTest")
{
    aRobotPositionShared_ = ARobotPositionShared::create();
    asservdriver_ = AAsservDriver::create("OPOS6UL_Robot", aRobotPositionShared_);
}

void test::AsservDriverManualTest::suite()
{
    this->testSet();
}

void test::AsservDriverManualTest::testSet()
{
    // Test désactivé : utilisait des méthodes deprecated supprimées
    // (getLeftInternalEncoder, setMotorLeftPosition, etc.)
    logger().info() << "testSet SKIPPED (deprecated methods removed)" << logs::end;
}
