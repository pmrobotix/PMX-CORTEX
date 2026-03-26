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
    logger().info() << "testSet..." << logs::end;

    asservdriver_->motion_ActivateManager(true);

    int power = 800;
    int timems = 3000;
    utils::Chronometer chrono("testSet");

    asservdriver_->resetEncoders();
    long left = asservdriver_->getLeftInternalEncoder();
    long right = asservdriver_->getRightInternalEncoder();

    asservdriver_->setMotorLeftPower(power, timems);
    asservdriver_->setMotorRightPower(power, timems);
    chrono.start();
    while (chrono.getElapsedTimeInMilliSec() < timems + 1000) {
        left = asservdriver_->getLeftInternalEncoder();
        right = asservdriver_->getRightInternalEncoder();
        logger().info() << "left=" << left << " right=" << right
                << " timems=" << chrono.getElapsedTimeInMilliSec() << logs::end;
        usleep(90000);
    }

    asservdriver_->setMotorLeftPosition(power, 300);
    asservdriver_->setMotorRightPosition(power, 300);
    chrono.start();
    while (chrono.getElapsedTimeInMilliSec() < timems + 2000) {
        left = asservdriver_->getLeftInternalEncoder();
        right = asservdriver_->getRightInternalEncoder();
        logger().info() << "left=" << left << " right=" << right
                << " timems=" << chrono.getElapsedTimeInMilliSec() << logs::end;
        usleep(90000);
    }

    logger().info() << "testSet OK" << logs::end;
}
