/*!
 * \file
 * \brief Implémentation de la classe O_Asserv_CalageTest.
 */

#include "O_Asserv_CalageTest.hpp"

#include <cmath>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include "action/Sensors.hpp"
#include "utils/Arguments.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_Asserv_CalageTest::configureConsoleArgs(int argc, char **argv)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.getArgs().addArgument("d", "dist en mm");
    robot.getArgs().addArgument("ty",
            "type de calage [B]ordure, [R]ight sensor, [L]eft sensor, [DR] demo right, [DL] demo left", "B");

    //reparse arguments
    robot.parseConsoleArgs(argc, argv);
}

void O_Asserv_CalageTest::run(int argc, char **argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
    configureConsoleArgs(argc, argv);
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Arguments args = robot.getArgs();

    float d = 0.0;
    if (args["d"] != "0") {
        d = atof(args["d"].c_str());
        logger().info() << "Arg d set " << args["d"] << ", d = " << d << logs::end;
    }

    std::string type;
    if (args["ty"] != "0") {
        type = args["ty"];
        logger().info() << "Arg type set " << args["ty"] << ", type = " << type << logs::end;
    }
    if (type == "B") { //calage bordure

        logger().info() << "Start Asserv " << logs::end;
        robot.setMyColor(PMXYELLOW);
        // setPositionAndColor AVANT startMotionTimerAndOdo
        robot.asserv().setPositionAndColor(70, 450.0, 0.0, (robot.getMyColor() != PMXYELLOW));
        robot.asserv().startMotionTimerAndOdo(false);
        robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
        robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "p= " << p.x << " " << p.y << " mm " << p.theta * 180.0f / M_PI << "° "
                << p.asservStatus << logs::end;
        robot.svgPrintPosition();

        logger().info() << "GO distance calage mm=" << d << logs::end;
        robot.asserv().calage(d, 30);
        logger().info() << "p= " << p.x << " " << p.y << " mm " << p.theta * 180.0f / M_PI << "° "
                << p.asservStatus << logs::end;
        robot.svgPrintPosition();

    } else if (type == "R") { //right sensor

        robot.setMyColor(PMXYELLOW);
        // setPositionAndColor AVANT startMotionTimerAndOdo
        robot.asserv().setPositionAndColor(70, 450.0, 0.0, (robot.getMyColor() != PMXYELLOW));
        robot.asserv().startMotionTimerAndOdo(true);

        { Navigator nav(&robot); nav.moveForwardToAndRotateAbsDeg(1680, 255, 0); }

        float mesure_mm = d;

        float pos_x_start_mm = 70;
        float pos_y_start_mm = 450;
        float delta_j_mm = 100;
        float delta_k_mm = 0;
        float robot_size_l_mm = 150;
        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "POS before : x=" << p.x << " y=" << p.y << " a=" << p.theta << " degrees="
                << p.theta * 180 / M_PI << logs::end;

        int succeed = robot.asserv().adjustRealPosition(pos_x_start_mm, pos_y_start_mm, p, delta_j_mm, delta_k_mm,
                mesure_mm, robot_size_l_mm);
        sleep(1);
        ROBOTPOSITION pnew = robot.asserv().pos_getPosition();
        logger().info() << "succeed=" << succeed << " POS after : x=" << pnew.x  << " y=" << pnew.y
                << " a=" << pnew.theta << " degrees=" << pnew.theta * 180 / M_PI << logs::end;
    } else

    if (type == "L") { //left sensor

        robot.setMyColor(PMXBLUE);
        // setPositionAndColor AVANT startMotionTimerAndOdo
        robot.asserv().setPositionAndColor(70, 450.0, 0.0, (robot.getMyColor() != PMXYELLOW));
        robot.asserv().startMotionTimerAndOdo(true);

        { Navigator nav(&robot); nav.moveForwardToAndRotateAbsDeg(1680, 255, 0); }

        float mesure_mm = d;

        float pos_x_start_mm = 70;
        float pos_y_start_mm = 450;
        float delta_j_mm = 100;
        float delta_k_mm = 0;
        float robot_size_l_mm = 150;
        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "POS before : x=" << p.x  << " y=" << p.y  << " a=" << p.theta << " degrees="
                << p.theta * 180 / M_PI << logs::end;

        int succeed = robot.asserv().adjustRealPosition(pos_x_start_mm, pos_y_start_mm, p, delta_j_mm, delta_k_mm,
                mesure_mm, robot_size_l_mm);
        sleep(1);
        ROBOTPOSITION pnew = robot.asserv().pos_getPosition();
        logger().info() << "succeed=" << succeed << " POS after : x=" << pnew.x << " y=" << pnew.y
                << " a=" << pnew.theta << " degrees=" << pnew.theta * 180 / M_PI << logs::end;
    } else

    if (type == "DR") {            //demo theorique sensor right

        robot.setMyColor(PMXYELLOW);
        // setPositionAndColor AVANT startMotionTimerAndOdo
        robot.asserv().setPositionAndColor(470, 1000.0, 0.0, (robot.getMyColor() != PMXYELLOW));
        robot.asserv().startMotionTimerAndOdo(true);

        { Navigator nav(&robot); nav.moveForwardToAndRotateAbsDeg(1780, 280, 0); }

        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "POS before : x=" << p.x << " y=" << p.y << " a=" << p.theta << " degrees="
                << p.theta * 180 / M_PI << logs::end;

        float mesure_mm = d;

        float pos_x_start_mm = 470;
        float pos_y_start_mm = 1000;
        float delta_j_mm = 200;
        float delta_k_mm = 100;
        float robot_size_l_mm = 150;

        int succeed = robot.asserv().adjustRealPosition(pos_x_start_mm, pos_y_start_mm, p, delta_j_mm, delta_k_mm,
                mesure_mm, robot_size_l_mm);

        ROBOTPOSITION pnew = robot.asserv().pos_getPosition();

        logger().info() << "succeed=" << succeed << " POS after : x=" << pnew.x << " y=" << pnew.y
                << " a=" << pnew.theta << " degrees=" << pnew.theta * 180 / M_PI << logs::end;
    } else

    if (type == "DL") { //demo theorique sensor left

        robot.setMyColor(PMXBLUE);
        // setPositionAndColor AVANT startMotionTimerAndOdo
        robot.asserv().setPositionAndColor(470, 1000.0, 0.0, (robot.getMyColor() != PMXYELLOW));
        robot.asserv().startMotionTimerAndOdo(true);

        { Navigator nav(&robot); nav.moveForwardToAndRotateAbsDeg(1780, 280, 0); }

        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "POS before : x=" << p.x << " y=" << p.y << " a=" << p.theta << " degrees="
                << p.theta * 180 / M_PI << logs::end;

        float mesure_mm = d;

        float pos_x_start_mm = 470;
        float pos_y_start_mm = 1000;
        float delta_j_mm = 200;
        float delta_k_mm = 100;
        float robot_size_l_mm = 150;

        int succeed = robot.asserv().adjustRealPosition(pos_x_start_mm, pos_y_start_mm, p, delta_j_mm, delta_k_mm,
                mesure_mm, robot_size_l_mm);

        ROBOTPOSITION pnew = robot.asserv().pos_getPosition();

        logger().info() << "succeed=" << succeed << " POS after : x=" << pnew.x << " y=" << pnew.y
                << " a=" << pnew.theta << " degrees=" << pnew.theta * 180 / M_PI << logs::end;
    }

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
