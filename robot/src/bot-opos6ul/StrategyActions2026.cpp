/*!
 * \file
 * \brief Enregistrement des actions et zones pour la Coupe de France 2026.
 *
 * Contient :
 *  - `registerStrategyActions2026`    : mapping JSON action_id -> methodes materielles.
 *  - `setupActivitiesZone2026`        : declaration des zones ia_createZone + ia_addAction.
 *  - les callbacks d'action de zone   : `end_of_match_top`, `push_prise_bas` (statics).
 *
 * Tout est regroupe ici pour que la migration annee+1 se limite a dupliquer
 * ce fichier et adapter aux nouvelles regles.
 */

#include "StrategyActions2026.hpp"

#include <chrono>
#include <thread>

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "navigator/Navigator.hpp"
#include "navigator/RetryPolicy.hpp"
#include "utils/Chronometer.hpp"

#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

namespace {

const logs::Logger& logger()
{
    static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyActions2026");
    return instance;
}

// =============================================================================
// Callbacks de zone (appeles via IAbyPath::ia_addAction)
// =============================================================================

/*!
 * \brief Action de fin de match : rejoint la zone de fin en haut de table.
 *
 * Attend la fin du chrono (96s), puis avance pour marquer les points de presence.
 * \return true si termine correctement, false si collision non resolue.
 */
bool end_of_match_top()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    logger().info() << __FUNCTION__ << logs::end;
    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.lastAction(true);
    robot.asserv().setMaxSpeed(true, 40);
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

    logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;

    robot.ia().iAbyPath().goToZone("zone_end_top", &zone);
    robot.displayPoints();

    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;
    ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), RetryPolicy::patient());
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " zone_end_top  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return false;
    }
    robot.svgPrintPosition();

    // attente de 96 sec
    while (robot.chrono().getElapsedTimeInSec() <= 96) {
        utils::sleep_for_secs(1);
    }

    ts = nav.line(451);
    robot.svgPrintPosition();

    robot.points += 20;
    robot.displayPoints();
    robot.svgPrintPosition();

    return true;
}

/*!
 * \brief Action de poussee de la prise en zone basse.
 *
 * Sequence composite : ralentit, ignore la detection, rejoint zone_prise_bas,
 * pousse jusqu'a (775, 200). Exposee comme MANIPULATION (`action_id:
 * "push_prise_bas"`) dans le JSON de strategie.
 *
 * \return true si termine, false si collision non resolue (remontee au runner
 *         qui abort l'instruction courante).
 */
bool push_prise_bas()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    logger().info() << __FUNCTION__ << logs::end;
    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.asserv().setMaxSpeed(true, 40);
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
    logger().info() << __FUNCTION__ << " start push_prise_bas x=" << zone.x << " y=" << zone.y << logs::end;
    robot.ia().iAbyPath().goToZone("zone_prise_bas", &zone);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    RetryPolicy policyPrise = { 1000000, 30, 30, 0, 0, false };

    ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), policyPrise);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " zone_prise_bas  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return false;
    }
    robot.svgPrintPosition();

    RetryPolicy policyPush = { 1000000, 10, 10, 0, 0, false };
    ts = nav.moveForwardTo(775, 200, policyPush);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " 775, 200  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return true;
    }
    robot.svgPrintPosition();
    return true;
}

// =============================================================================
// Declaration des zones (ia_createZone + ia_addAction)
// =============================================================================

void setupZonesHomologation(OPOS6UL_RobotExtended& robot)
{
    logger().info() << "setupActivitiesZone2026 : strategy=all (homologation)" << logs::end;
    logger().debug() << "color = " << robot.getMyColor() << logs::end;

    robot.ia().iAbyPath().ia_createZone("zone_end_top",    150, 1550, 450, 450, 350, 1100,  90);
    robot.ia().iAbyPath().ia_createZone("zone_start",     1000,    0, 450, 450, 1300, 400,  90);
    robot.ia().iAbyPath().ia_createZone("zone_prise_bas",  550,    0, 450, 100,  775, 550, -90);

    robot.ia().iAbyPath().ia_addAction("end_of_match_top", &end_of_match_top);
}

void setupZonesTableTest(OPOS6UL_RobotExtended& robot)
{
    logger().info() << "setupActivitiesZone2026 : strategy=tabletest (layout decale -420mm)"
                    << logs::end;
    logger().debug() << "color = " << robot.getMyColor() << logs::end;

    robot.ia().iAbyPath().ia_createZone("zone_end_top",    150, 1550 - 420, 450, 450,  350, 1100 - 420,  90);
    robot.ia().iAbyPath().ia_createZone("zone_start",     1000,         0,  450, 450, 1300,        400,  90);
    robot.ia().iAbyPath().ia_createZone("zone_prise_bas",  550,         0,  450, 100,  775,        550, -90);

    robot.ia().iAbyPath().ia_addAction("end_of_match_top", &end_of_match_top);
}

} // namespace

// =============================================================================
// API publique
// =============================================================================

void setupActivitiesZone2026(OPOS6UL_RobotExtended& robot, const std::string& strategy)
{
    if (strategy == "tabletest") {
        setupZonesTableTest(robot);
    } else if (strategy == "all") {
        setupZonesHomologation(robot);
    } else {
        logger().error() << "strategy '" << strategy << "' inconnue (attendu: all|tabletest)"
                         << logs::end;
    }
}

void registerStrategyActions2026(ActionRegistry& registry, OPOS6UL_RobotExtended& robot)
{
    // --- Banderole (action de fin de match) ---
    registry.registerAction("banderole",
        [&robot]() { robot.actions().ax12_GO_banderole(); return true; });
    registry.registerAction("banderole_init",
        [&robot]() { robot.actions().ax12_init_banderole(); return true; });

    // --- Bras lateraux ---
    registry.registerAction("bras_droit",
        [&robot]() { robot.actions().ax12_bras_droit(); return true; });
    registry.registerAction("bras_droit_init",
        [&robot]() { robot.actions().ax12_bras_droit_init(); return true; });
    registry.registerAction("bras_gauche",
        [&robot]() { robot.actions().ax12_bras_gauche(); return true; });
    registry.registerAction("bras_gauche_init",
        [&robot]() { robot.actions().ax12_bras_gauche_init(); return true; });

    // --- Reset global des AX12 ---
    registry.registerAction("init_all",
        [&robot]() { robot.actions().ax12_init(); return true; });

    // --- Sequences composites (deplacement + manipulation) ---
    // Les actions ci-dessous combinent mouvement + reglages capteurs + retry
    // specifique. Plus simple en C++ qu'en enchainement de tasks JSON
    // (surtout pour les cas "push" qui desactivent toute la detection).
    registry.registerAction("push_prise_bas",
        [](){ return push_prise_bas(); });

    // TODO 2026 : ajouter ici les actions specifiques au reglement quand les
    // elements de jeu (plantes, pots, garde-mangers, caisses...) et leurs
    // mecanismes seront figes.
}
