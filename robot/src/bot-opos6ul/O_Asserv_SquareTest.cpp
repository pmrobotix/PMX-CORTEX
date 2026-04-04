/*!
 * \file
 * \brief Implémentation de la classe O_Asserv_SquareTest.
 */

#include "O_Asserv_SquareTest.hpp"

#include <cstdlib>
#include <string>

#include "action/Sensors.hpp"
#include "utils/Arguments.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_Asserv_SquareTest::configureConsoleArgs(int argc, char** argv)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    robot.getArgs().addArgument("x", "x mm");
    robot.getArgs().addArgument("y", "y mm");
    robot.getArgs().addArgument("d", "segment mm");
    robot.getArgs().addArgument("nb", "nbre de tours", "1");

    //reparse arguments
    robot.parseConsoleArgs(argc, argv);
}

void O_Asserv_SquareTest::run(int argc, char** argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
    configureConsoleArgs(argc, argv);

    utils::Chronometer chrono("O_Asserv_SquareTest");
    int left;
    int right;

    float x = 0.0;
    float y = 0.0;
    float d = 0.0;
    int nb = 0;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    Arguments args = robot.getArgs();

    if (args["x"] != "0") {
        x = atof(args["x"].c_str());
        logger().info() << "Arg x set " << args["x"] << ", x = " << x << logs::end;
    }
    if (args["y"] != "0") {
        y = atof(args["y"].c_str());
        logger().info() << "Arg y set " << args["y"] << ", y = " << y << logs::end;
    }
    if (args["d"] != "0") {
        d = atof(args["d"].c_str());
        logger().info() << "Arg d set " << args["d"] << ", d = " << d << logs::end;
    }

    if (args["nb"] != "0") {
        nb = atoi(args["nb"].c_str());
        logger().info() << "Arg nb set " << args["nb"] << ", nb = " << nb << logs::end;
    }

    robot.setMyColor(PMXYELLOW);

    robot.asserv().startMotionTimerAndOdo(true);

    robot.asserv().setPositionAndColor(0.0, 300.0, 0.0, (robot.getMyColor() != PMXYELLOW));
    robot.svgPrintPosition();

    robot.actions().start();
    robot.actions().sensors().setIgnoreFrontNearObstacle(false, false, false);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
    robot.actions().sensors().addTimerSensors(200);

    chrono.start();

    for (int n = 1; n <= nb; n++) {
        logger().info() << "moveForwardTo(" << x << ", " << y << ")" << logs::end;
        robot.asserv().moveForwardTo(x, y);

        robot.asserv().getEncodersCounts(&right, &left);
        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "time= "
                << robot.chrono().getElapsedTimeInMilliSec()
                << "ms ; left= " << left << " ; right= " << right
                << " x=" << p.x << " y=" << p.y
                << " deg=" << p.theta * 180.0 / M_PI
                << logs::end;

        robot.svgPrintPosition();

        logger().info() << "moveForwardTo(" << x + d << ", " << y << ")" << logs::end;
        robot.asserv().moveForwardTo(x + d, y);

        robot.asserv().getEncodersCounts(&right, &left);
        p = robot.asserv().pos_getPosition();
        logger().info() << "time= "
                << robot.chrono().getElapsedTimeInMilliSec()
                << "ms ; left= " << left << " ; right= " << right
                << " x=" << p.x << " y=" << p.y
                << " deg=" << p.theta * 180.0 / M_PI
                << logs::end;

        robot.svgPrintPosition();

        logger().info() << "moveForwardTo(" << x + d << ", " << y + d << ")" << logs::end;
        robot.asserv().moveForwardTo(x + d, y + d);

        robot.asserv().getEncodersCounts(&right, &left);
        p = robot.asserv().pos_getPosition();
        logger().info() << "time= "
                << robot.chrono().getElapsedTimeInMilliSec()
                << "ms ; left= " << left << " ; right= " << right
                << " x=" << p.x << " y=" << p.y
                << " deg=" << p.theta * 180.0 / M_PI
                << logs::end;

        robot.svgPrintPosition();

        logger().info() << "moveForwardTo(" << x << ", " << y + d << ")" << logs::end;
        robot.asserv().moveForwardTo(x, y + d);

        robot.asserv().getEncodersCounts(&right, &left);
        p = robot.asserv().pos_getPosition();
        logger().info() << "time= "
                << robot.chrono().getElapsedTimeInMilliSec()
                << "ms ; left= " << left << " ; right= " << right
                << " x=" << p.x << " y=" << p.y
                << " deg=" << p.theta * 180.0 / M_PI
                << logs::end;

        robot.svgPrintPosition();

        logger().info() << "moveForwardAndRotateTo(" << x << ", " << y << ", 0.0)" << logs::end;
        robot.asserv().moveForwardAndRotateTo(x, y, 0.0);

        robot.asserv().getEncodersCounts(&right, &left);
        p = robot.asserv().pos_getPosition();
        logger().info() << "time= "
                << robot.chrono().getElapsedTimeInMilliSec()
                << "ms ; left= " << left << " ; right= " << right
                << " x=" << p.x << " y=" << p.y
                << " deg=" << p.theta * 180.0 / M_PI
                << logs::end;

        robot.svgPrintPosition();
    }

    logger().info() << "End time= "
            << chrono.getElapsedTimeInMilliSec()
            << " ; x=" << robot.asserv().pos_getX_mm()
            << " y=" << robot.asserv().pos_getY_mm()
            << " degrees=" << robot.asserv().pos_getThetaInDegree()
            << logs::end;

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
