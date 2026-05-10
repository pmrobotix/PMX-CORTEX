/*!
 * \file
 * \brief Implémentation de la classe O_State_WaitEndOfMatch.
 */

#include "O_State_WaitEndOfMatch.hpp"

#include <unistd.h>

#include "action/LcdShield.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"
#include "O_State_DecisionMakerIA.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

IAutomateState* O_State_WaitEndOfMatch::execute(Robot&)
{

    logger().info() << "O_State_WaitEndOfMatch executing..." << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    //démarrage du chrono et des taches du decision maker
    logger().info() << "Start Chronometer" << logs::end;
    robot.chrono().start();

    while (robot.chrono().getElapsedTimeInSec() <= 99) {
        if (g_shutdownRequested.load(std::memory_order_relaxed)) break;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        long time = robot.chrono().getElapsedTimeInSec();
        this->logger().info() << "O_State_Wait90SecAction::execute chrono " << time << logs::end;
        robot.actions().lcd2x16().setCursor(0, 1);
        robot.actions().lcd2x16().print(time);
        robot.actions().lcd2x16().print(" sec");
        //indique le temps en binaire
        //robot.actions().ledBar().flash(time, LED_GREEN);
    }

    //release dans tous les cas
//    robot.actions().ax12_left_cil_release(-1);
//    robot.actions().ax12_left_cil_release(-1);
//    robot.actions().ax12_left_cil_release(-1);

    //robot.actions().funny_action_deploy(2000);

    //robot.points += 5;
    robot.displayPoints();
    robot.actions().sensors().display(robot.points);
    robot.actions().sensors().display(robot.points);
/*
    //TODO danse de fin ?
    robot.actions().ax12_bras_droit(0);
    robot.actions().ax12_bras_gauche(-1);
    robot.actions().ax12_bras_droit_init(0);
    robot.actions().ax12_bras_gauche_init(-1);
    robot.actions().ax12_bras_droit(0);
    robot.actions().ax12_bras_gauche(-1);

    robot.actions().ax12_bras_droit(0);
    robot.actions().ax12_bras_gauche(-1);
    robot.actions().ax12_bras_droit_init(0);
    robot.actions().ax12_bras_gauche_init(-1);
    robot.actions().ax12_bras_droit(0);
    robot.actions().ax12_bras_gauche(-1);
    robot.actions().ax12_bras_droit_init(0);
    robot.actions().ax12_bras_gauche_init(-1);*/

    this->logger().info() << "O_State_Wait90SecAction::stopping... "
            << robot.chrono().getElapsedTimeInSec() << logs::end;
    robot.freeMotion();

    robot.end90s(true); //indique que l'action est effectuée au prog princ
    logger().info() << "cancel decisionmaker" << logs::end;
    robot.decisionMaker_->cancel();

    //robot.asserv().stopMotors(); //TODO to be tested
    robot.freeMotion();
    robot.actions().releaseAll();

    robot.stopExtraActions(); //stop specific actions, sensors, can take time for servos...


    robot.svgPrintEndOfFile();

    logger().info() << "Display Points after 100sec" << logs::end;

    robot.actions().lcd2x16().clear();

    robot.actions().lcd2x16().setBacklightOn();
    robot.actions().lcd2x16().setCursor(0, 0);
    robot.actions().lcd2x16().print(robot.points);
    robot.actions().lcd2x16().print(" points ?");

    robot.actions().lcd2x16().setCursor(0, 1);
    robot.actions().lcd2x16().print("ooooxxyyyy !");

    robot.actions().ledBar().flashAll(LED_GREEN);

    // Animation K2000 en arriere-plan : le timer tourne dans le scheduler,
    // le main thread reste libre pour scruter les boutons. Grand nb de
    // cycles (1000) car on l'arrete explicitement avec stop() a la sortie.
    robot.actions().ledBar().startTimerK2mil(1000, 50000, LED_GREEN, false);

    ButtonTouch b = BUTTON_NONE;
    while (1) {
        if (g_shutdownRequested.load(std::memory_order_relaxed)) {
            logger().info() << "Shutdown requested (SIGINT)" << logs::end;
            break;
        }

        b = robot.actions().buttonBar().checkOneOfAllPressed();
        if (b == BUTTON_BACK_KEY) {
            logger().info() << "BUTTON_BACK_KEY" << logs::end;
            break;
        }
        if (b == BUTTON_ENTER_KEY) {
            logger().info() << "BUTTON_ENTER_KEY" << logs::end;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Arret de l'animation K2000 avant cleanup final.
    robot.actions().ledBar().stop();

    logger().info() << "O_State_WaitEndOfMatch executed " << robot.chrono().getElapsedTimeInSec() << " sec"
            << logs::end;

    // Sortie immediate du processus : flush des logs puis _exit().
    // _exit() est async-signal-safe et ne lance pas les destructeurs (donc
    // pas de double stopExtraActions ni de pthread_join bloquant sur le
    // decisionMaker dans OPOS6UL_RobotExtended::begin()). Le cleanup
    // critique (freeMotion, releaseAll, stopExtraActions, svgPrintEndOfFile)
    // a deja ete realise plus haut, juste apres le timeout 99s.
    logs::LoggerFactory::instance().stopLog();
    _exit(0);

    return NULL; //finish all state (jamais atteint apres _exit)
}
