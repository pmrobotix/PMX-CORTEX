/*!
 * \file
 * \brief Implémentation de la classe O_AsservCalibrationTest.
 *
 * Argument positionnel `step` (0..5) : pas de slash, juste le numéro.
 *
 * === Exemples ===
 *
 *   ./bot-opos6ul cal 0   // set position fixe puis lit pos + codeurs en boucle
 *   ./bot-opos6ul cal 1   // codeurs seuls : pousser le robot a la main, voir
 *                         // si left/right bougent (test liaison Nucleo->OPO)
 *   ./bot-opos6ul cal 2   // moteurs L+R a 25% pendant ~5s, log pos + codeurs
 *                         // (test commande directe OPO->Nucleo, bypass asserv)
 *   ./bot-opos6ul cal 3   // assistedHandling en boucle (regler P : pousser,
 *                         // le robot doit resister/revenir)
 *   ./bot-opos6ul cal 4   // line(100) en assisted (regler D en translation)
 *   ./bot-opos6ul cal 5   // rotateDeg(90) en assisted (regler D en rotation)
 *
 * Note : `cal /step 4` NE marche pas — le parseur lit `/s` (strategy) et prend
 * "4" comme nom de strategie -> cherche strategy4.json -> abort. Ne pas mettre
 * de slash devant `step`.
 */

#include "O_AsservCalibrationTest.hpp"

#include <cstdlib>
#include <unistd.h>
#include <string>

#include "utils/Arguments.hpp"
#include "Robot.hpp"
#include "navigator/Navigator.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_AsservCalibrationTest::configureConsoleArgs(int argc, char** argv)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.getArgs().addArgument("step", "n step test");

    //reparse arguments
    robot.parseConsoleArgs(argc, argv);
}

void O_AsservCalibrationTest::run(int argc, char** argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
    configureConsoleArgs(argc, argv);
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    int left = 0;
    int right = 0;
    int nb = 0;
    int step = 0;

    Arguments args = robot.getArgs();

    if (args["step"] != "0") {
        step = atoi(args["step"].c_str());
        logger().debug() << "Arg step set " << args["step"] << ", step = " << step << logs::end;
    }

    utils::Chronometer chrono("O_AsservCalibrationTest");

    robot.asserv().setPositionAndColor(100.0, 800.0, 0.0, robot.isMatchColor());
    robot.asserv().startMotionTimerAndOdo(true);

    robot.svgPrintPosition();

    chrono.start();

    if (step == 0) {
        //set position
        logger().info() << "set position..." << logs::end;
        robot.asserv().setPositionAndColor(300, 500, 0.0, robot.isMatchColor());

        while (1) {

            robot.asserv().getEncodersCounts(&right, &left);
            ROBOTPOSITION p = robot.asserv().pos_getPosition();
            logger().info() << "time= "
                    << robot.chrono().getElapsedTimeInMilliSec()
                    << "ms ; left= " << left << " ; right= " << right
                    << " x=" << p.x << " y=" << p.y
                    << " deg=" << p.theta * 180.0 / M_PI
                    << logs::end;
            utils::sleep_for_micros(100000);
            nb++;
        }
    }

    if (step == 1) {
        logger().info() << "ETAPE 1 : TEST CODEURS - compter sur un metre" << logs::end;
        while (1) {

            robot.asserv().getEncodersCounts(&right, &left);
            ROBOTPOSITION p = robot.asserv().pos_getPosition();
            logger().info() << nb
                    << " time= "
                    << robot.chrono().getElapsedTimeInMilliSec()
                    << "ms ; left(2)= " << left << " ; right(1)= " << right
                    << "\tx=" << p.x << " y=" << p.y
                    << " deg=" << p.theta * 180.0 / M_PI
                    << logs::end;
            utils::sleep_for_micros(100000);
            nb++;
        }
    }

    if (step == 2) {
        logger().info() << "ETAPE 2 : TEST MOTEURS ET CODEURS" << logs::end;
        while (1) {
            robot.asserv().runMotorLeft(25, 0);
            robot.asserv().runMotorRight(25, 0);

            robot.asserv().getEncodersCounts(&right, &left);
            ROBOTPOSITION p = robot.asserv().pos_getPosition();
            logger().info() << "time= "
                    << robot.chrono().getElapsedTimeInMilliSec()
                    << "ms ; left= " << left << " ; right= " << right
                    << " x=" << p.x << " y=" << p.y
                    << " deg=" << p.theta * 180.0 / M_PI
                    << logs::end;
            usleep(100000);
            nb++;
            if (nb > 50) break;
        }
    }

    if (step == 3) {
        logger().info() << "ETAPE 3 : assistedHandling pour regler P" << logs::end;
        while (1) {
            robot.asserv().assistedHandling();
            sleep(1);
        }
    }

    if (step == 4) {
        logger().info() << "ETAPE 4 : on avance pour regler D" << logs::end;
        robot.asserv().assistedHandling();
        robot.asserv().line(100);
        sleep(1);
    }

    if (step == 5) {
        logger().info() << "ETAPE 5 : on tourne pour regler D" << logs::end;
        robot.asserv().assistedHandling();
        Navigator nav(&robot);
        nav.rotateDeg(90);
        sleep(1);
    }

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
