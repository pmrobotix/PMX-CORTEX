/*!
 * \file
 * \brief Test fonctionnel des capteurs de detection (balise beacon).
 *
 * Log les positions adversaires, niveaux de detection,
 * timing beacon (beacon_seq, beacon_delay_us) et position adversaire.
 */

#include "O_SensorsTest.hpp"

#include <chrono>
#include <string>
#include <thread>

#include "action/Sensors.hpp"
#include "interface/ASensorsDriver.hpp"
#include "geometry/DetectionEvent.hpp"
#include "Robot.hpp"
#include "asserv/Asserv.hpp"
#include "utils/Chronometer.hpp"
#include "utils/PointerList.hpp"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

using namespace std;

void O_SensorsTest::run(int argc, char** argv) {
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.setMyColor(PMXYELLOW);
    //robot.asserv().startMotionTimerAndOdo(false); //assistedHandling is enabled with "true" !
    utils::Chronometer chrono("O_SensorsTest");
    chrono.start();
    int front = 0, back = 0, left=0, right=0;
    ASensorsDriver::bot_positions vadv;

/*
    //TEST en direct
    while (chrono.getElapsedTimeInSec() < 200) {
        int err = robot.actions().sensors().sync("beacon_sync");

        vadv = robot.actions().sensors().getPositionsAdv();

        for (ASensorsDriver::bot_positions::size_type i = 0; i < vadv.size(); i++) {
            logger().info() << "TEST en direct : vadv nb=" << vadv.size()
        << " detected=" << vadv[i].nbDetectedBots
        << " x=" << vadv[i].x
        << " y=" << vadv[i].y
        << " a=" << vadv[i].theta
        << " d=" << vadv[i].d
                            << logs::end;
        }


        //std::this_thread::sleep_for(chrono::nanoseconds(100000000));
        utils::sleep_for_micros(200000);
        //usleep(100000);
    }
*/


    // ORDRE DE DEMARRAGE IMPORTANT :
    // 1. setPositionAndColor = reference de match (couleur + position initiale).
    //    Rien ne doit fonctionner avant : le set position remet tout a zero
    //    cote Nucleo. Avant ce setPos, les positions recues du thread CBOR
    //    sont les residus de la session precedente -> filtre par positionInitialized_.
    robot.asserv().setPositionAndColor(200.0, 200.0, 0.0,(bool)(robot.getMyColor() != PMXYELLOW));

    // 2. Demarrer le thread CBOR (reception des positions Nucleo en continu)
    //    La methode inclut une attente 50ms pour laisser la Nucleo appliquer le setPos.
    robot.asserv().startMotionTimerAndOdo(false);

    robot.svgPrintPosition();

    ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << "p= " << p.x  << " " << p.y << " mm " << p.theta * 180.0f / M_PI << "° "
                << p.asservStatus << logs::end;

//    robot.actions().sensors().setIgnoreAllFrontNearObstacle(false);
//    robot.actions().sensors().setIgnoreAllBackNearObstacle(true);


    robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

    // DEBUG : ne pas filtrer les detections hors table pour voir la projection brute dans le SVG
    robot.actions().sensors().remove_outside_table(false);

    // 4. Demarrer les actions (scheduler) + timer beacon (detection adverse)
    robot.actions().start();
    robot.actions().sensors().addTimerSensors(20);
    robot.chrono().start();

    uint32_t last_seq = 0;
    while (chrono.getElapsedTimeInSec() < 20) {

        robot.svgPrintPosition();
        vadv = robot.actions().sensors().getPositionsAdv();

        const DetectionEvent& det = robot.actions().sensors().lastDetection();

        // Afficher seulement si nouvelles donnees beacon
        if (det.beacon_seq != 0 && det.beacon_seq != last_seq) {
            last_seq = det.beacon_seq;

            logger().info() << " t=" << chrono.getElapsedTimeInSec() << "s pos x="
                                << robot.asserv().pos_getX_mm()
                                << " y="
                                << robot.asserv().pos_getY_mm()
                                << " a="
                                << robot.asserv().pos_getThetaInDegree()
                                << logs::end;

            for (ASensorsDriver::bot_positions::size_type i = 0; i < vadv.size(); i++) {
                logger().info() << " vadv nb="
                        << vadv.size()
                        << " detected="
                        << vadv[i].nbDetectedBots
                        << " x="
                        << vadv[i].x
                        << " y="
                        << vadv[i].y
                        << " a="
                        << vadv[i].theta_deg
                        << " d="
                        << vadv[i].d
                        << logs::end;
            }

            logger().info() << " front=" << det.frontLevel << " back=" << det.backLevel
                    << " beacon_seq=" << det.beacon_seq
                    << " beacon_delay=" << det.beacon_delay_us << "us"
                    << " adv=(" << det.x_adv_mm << "," << det.y_adv_mm << ")"
                    << " valid=" << det.valid
                    << logs::end;
        }

        utils::sleep_for_micros(5000); // 5ms : suffisant pour ne rater aucun cycle beacon (~60ms)
    }

    //TEST par front recu




//    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
//    robot.setMyColor(PMXBLUE);//YELLOW
//    robot.asserv().startMotionTimerAndOdo(false); //assistedHandling is enabled with "true" !
//    robot.asserv().setPositionAndColor(400.0, 400.0, 0.0, (robot.getMyColor() != PMXVIOLET));//YELLOW
//    RobotPosition p = robot.asserv().pos_getPosition();
//    logger().info() << "p= " << p.x * 1000.0 << " " << p.y * 1000.0 << " mm " << p.theta * 180.0f / M_PI << "° "
//            << p.asservStatus << logs::end;
//
//    //execution sans le actionTimer
//    int front, back, left, right, mright, mleft;
//    utils::Chronometer chrono("O_SensorsTest");
//    chrono.start();
//    robot.actions().sensors().setIgnoreAllFrontNearObstacle(false);
//    robot.actions().sensors().setIgnoreAllBackNearObstacle(false);
//    /*
//     while (chrono.getElapsedTimeInSec() < 100) {
//     right = robot.actions().sensors().rightSide();
//     left = robot.actions().sensors().leftSide();
//     front = robot.actions().sensors().front(true);
//     back = robot.actions().sensors().back(true);
//
//     mright = robot.actions().sensors().multipleRightSide(10);
//     mleft = robot.actions().sensors().multipleLeftSide(10);
//
//     usleep(100000);
//     logger().info() << " front=" << front << "   back=" << back << "                       left=" << left
//     << "   right=" << right << logs::end;
//     logger().info() << "                                         " << "                       mleft=" << mleft << "   mright=" << mright<< logs::end;
//     logger().info() << logs::end;
//     }*/
//
//    //detection adverse
//    robot.actions().start();
//    robot.actions().sensors().addTimerSensors(50);
//
//    while (chrono.getElapsedTimeInSec() < 200) {
//        front = robot.actions().sensors().front(true);
//        back = robot.actions().sensors().back(true);
//
//        mright = robot.actions().sensors().multipleRightSide(10);
//        mleft = robot.actions().sensors().multipleLeftSide(10);
//
//        usleep(1000000);
//        logger().info() << " front=" << front << " back=" << back << "                       mleft=" << mleft << "   mright=" << mright
//                << logs::end;
//    }

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}

