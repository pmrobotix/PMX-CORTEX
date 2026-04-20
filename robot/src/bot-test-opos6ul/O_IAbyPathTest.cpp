/*!
 * \file
 * \brief Implementation de la classe O_IAByPathTest.
 */

#include "O_IAbyPathTest.hpp"

#include <pmr_playground.h>
#include <string>

#include "action/Sensors.hpp"
#include "interface/AAsservDriver.hpp"
#include "ia/IAbyPath.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

using namespace std;

bool O_IAbyPathTest_action1()
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    robot.logger().info() << "start action1." << logs::end;

    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.ia().iAbyPath().goToZone("zone_1", &zone);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    ts = nav.pathToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta));
    if (ts != TRAJ_FINISHED) {
        robot.logger().info() << "________action1 ts=" << ts << logs::end;
        return false;
    }

    robot.svgPrintPosition();

    robot.logger().info() << "action1 done." << logs::end;

    return true;
}

bool O_IAByPathTest_action2()
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.logger().info() << "start action2." << logs::end;
    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.ia().iAbyPath().goToZone("zone_2", &zone);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    ts = nav.pathToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta));
    if (ts != TRAJ_FINISHED) {
        robot.logger().info() << "_________action2 ts=" << ts << logs::end;
        return false;
    }

    robot.svgPrintPosition();

    robot.logger().info() << "action2 done." << logs::end;
    return true;
}

void O_IAByPathTest::run(int argc, char** argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    // setPositionAndColor AVANT startMotionTimerAndOdo
    robot.asserv().setPositionAndColor(70, 190, 0.0, (robot.getMyColor() != PMXYELLOW));
    robot.asserv().startMotionTimerAndOdo(true);

    robot.asserv().assistedHandling();

    IASetup();
    initPlayground();

    Navigator nav(&robot, &robot.ia().iAbyPath());

    robot.svgPrintPosition();
    nav.line(155);
    robot.svgPrintPosition();

    robot.actions().start();
    robot.actions().sensors().startSensorsThread(20);

    robot.chrono().start();
    robot.ia().iAbyPath().ia_start(); //launch IA

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}

void O_IAByPathTest::IASetup()
{
    logger().debug() << "IASetup" << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    robot.ia().iAbyPath().ia_createZone("depart", 0, 0, 100, 100, 100, 100, 0);
    robot.ia().iAbyPath().ia_createZone("zone_1", 400, 650, 200, 100, 400, 650, 90);
    robot.ia().iAbyPath().ia_createZone("zone_2", 800, 400, 100, 100, 800, 400, -90);

    robot.ia().iAbyPath().ia_addAction("action1", &O_IAbyPathTest_action1);
    robot.ia().iAbyPath().ia_addAction("action2", &O_IAByPathTest_action2);
}

void O_IAByPathTest::initPlayground()
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    p_ = new SymmetricalPlayground(0.0, 0.0, 2000.0, 3200.0, 0.5, 1.0, 1000.0);

    p_->compute_edges();

    robot.ia().iAbyPath().addPlayground(p_);
    robot.ia().iAbyPath().toSVG();
}
