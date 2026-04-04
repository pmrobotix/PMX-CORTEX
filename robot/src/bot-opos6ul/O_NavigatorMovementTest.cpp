/*!
 * \file
 * \brief Implementation du test de non-regression Navigator.
 *
 * 8 boucles independantes, chacune revient a son point de depart.
 * Chaque boucle a un point de depart configurable (sx, sy).
 * Tous les waypoints sont en offset relatif par rapport a ce depart.
 * Voir O_NavigatorMovementTest.hpp pour la description des boucles.
 */

#include "O_NavigatorMovementTest.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "interface/AAsservDriver.hpp"
#include "Robot.hpp"
#include "log/Logger.hpp"
#include "navigator/Navigator.hpp"
#include "navigator/RetryPolicy.hpp"
#include "ia/IAbyPath.hpp"
#include <pmr_symmetrical_pg.h>
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

// =============================================================================
// Helpers
// =============================================================================

void O_NavigatorMovementTest::resetPosition(float x, float y, float angleDeg)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.asserv().setPositionAndColor(x, y, angleDeg, false);
}

bool O_NavigatorMovementTest::checkResult(TRAJ_STATE ts, TRAJ_STATE expected, const std::string& name)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    if (ts == expected)
    {
        passed_++;
        logger().info() << "  PASS " << name << " ts=" << robot.asserv().getTraj(ts) << logs::end;
        return true;
    }
    failed_++;
    logger().error() << "  FAIL " << name << " ts=" << robot.asserv().getTraj(ts)
                     << " expected=" << robot.asserv().getTraj(expected) << logs::end;
    return false;
}

bool O_NavigatorMovementTest::checkPosition(float expectedX, float expectedY, float tolerance, const std::string& name)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    float x = robot.asserv().pos_getX_mm();
    float y = robot.asserv().pos_getY_mm();
    float dx = x - expectedX;
    float dy = y - expectedY;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < tolerance)
    {
        passed_++;
        logger().info() << "  PASS " << name << " pos=(" << x << "," << y << ") dist=" << dist << logs::end;
        return true;
    }
    failed_++;
    logger().error() << "  FAIL " << name << " pos=(" << x << "," << y
                     << ") expected=(" << expectedX << "," << expectedY
                     << ") dist=" << dist << " tolerance=" << tolerance << logs::end;
    return false;
}

bool O_NavigatorMovementTest::checkAngle(float expectedDeg, float tolerance, const std::string& name)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    float deg = robot.asserv().pos_getThetaInDegree();
    float diff = deg - expectedDeg;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;

    if (std::fabs(diff) < tolerance)
    {
        passed_++;
        logger().info() << "  PASS " << name << " angle=" << deg << " diff=" << diff << logs::end;
        return true;
    }
    failed_++;
    logger().error() << "  FAIL " << name << " angle=" << deg
                     << " expected=" << expectedDeg
                     << " diff=" << diff << " tolerance=" << tolerance << logs::end;
    return false;
}

// =============================================================================
// Boucle 1 — Mouvements directs (bleu) — carre 100mm
// line, goTo, goToReverse, rotations — retour au depart
// =============================================================================

void O_NavigatorMovementTest::loop1_DirectMoves()
{
    float sx = 200, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 1 : Mouvements directs (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // line(100) vers droite
    ts = nav.line(100);
    checkResult(ts, TRAJ_FINISHED, "line(100)");
    checkPosition(sx + 100, sy, 5, "line -> +100,0");

    // goTo diagonale
    ts = nav.goTo(sx + 100, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "goTo(+100,+100)");
    checkPosition(sx + 100, sy + 100, 5, "goTo -> +100,+100");

    // goToReverse retour en X
    ts = nav.goToReverse(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "goToReverse(0,+100)");
    checkPosition(sx, sy + 100, 5, "goToReverse -> 0,+100");

    // rotateDeg(-90) puis line(100) pour revenir
    ts = nav.rotateDeg(-90);
    checkResult(ts, TRAJ_FINISHED, "rotateDeg(-90)");
    checkAngle(-90, 2, "angle -90");

    ts = nav.line(100);
    checkResult(ts, TRAJ_FINISHED, "line(100) retour");
    checkPosition(sx, sy, 5, "retour depart");

    // Rotations sur place
    ts = nav.rotateAbsDeg(45);
    checkResult(ts, TRAJ_FINISHED, "rotateAbsDeg(45)");
    checkAngle(45, 2, "angle 45");

    ts = nav.faceTo(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "faceTo(0,+100)");
    checkAngle(90, 2, "faceTo = 90");

    ts = nav.reverseFaceTo(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "reverseFaceTo(0,+100)");
    checkAngle(-90, 2, "reverseFaceTo = -90");

    ts = nav.rotateAbsDeg(sa);
    checkResult(ts, TRAJ_FINISHED, "retour angle initial");

    checkPosition(sx, sy, 5, "boucle 1 retour pos");
    checkAngle(sa, 2, "boucle 1 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 2 — Combinaisons (bleu) — carre 200mm
// goToAndRotateAbsDeg, goToAndRotateRelDeg, goToAndFaceTo — retour
// =============================================================================

void O_NavigatorMovementTest::loop2_Combinations()
{
    float sx = 800, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 2 : Combinaisons (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // goToAndRotateAbsDeg (+200,0, 90)
    ts = nav.goToAndRotateAbsDeg(sx + 200, sy, 90);
    checkResult(ts, TRAJ_FINISHED, "goToAndRotateAbsDeg(+200,0,90)");
    checkPosition(sx + 200, sy, 5, "(+200,0)");
    checkAngle(90, 2, "angle 90");

    // goToAndRotateRelDeg (+200,+200, -90) : angle = 90+(-90) = 0
    ts = nav.goToAndRotateRelDeg(sx + 200, sy + 200, -90);
    checkResult(ts, TRAJ_FINISHED, "goToAndRotateRelDeg(+200,+200,-90)");
    checkPosition(sx + 200, sy + 200, 5, "(+200,+200)");
    checkAngle(0, 2, "angle 0");

    // goToAndFaceTo (0,+200, face vers depart) = -90deg
    ts = nav.goToAndFaceTo(sx, sy + 200, sx, sy);
    checkResult(ts, TRAJ_FINISHED, "goToAndFaceTo(0,+200, face depart)");
    checkPosition(sx, sy + 200, 5, "(0,+200)");
    checkAngle(-90, 2, "face depart = -90");

    // retour
    ts = nav.goTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "retour depart");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 5, "boucle 2 retour pos");
    checkAngle(sa, 2, "boucle 2 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 3 — manualPath STOP (robot vert, pas de trait) — carre 200mm
// =============================================================================

void O_NavigatorMovementTest::loop3_ManualPathStop()
{
    float sx = 200, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 3 : manualPath STOP --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    ts = nav.manualPath({});
    checkResult(ts, TRAJ_FINISHED, "manualPath vide");

    ts = nav.manualPath(
        {{sx + 200, sy}, {sx + 200, sy + 200}, {sx, sy + 200}, {sx, sy}},
        RetryPolicy::noRetry(), STOP);
    checkResult(ts, TRAJ_FINISHED, "manualPath STOP carre 200mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 3 retour pos");
    checkAngle(sa, 2, "boucle 3 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 4 — manualPath CHAIN (vert continu) — carre 300mm
// =============================================================================

void O_NavigatorMovementTest::loop4_ManualPathChain()
{
    float sx = 800, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 4 : manualPath CHAIN (vert continu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts = nav.manualPath(
        {{sx + 300, sy}, {sx + 300, sy + 300}, {sx, sy + 300}, {sx, sy}},
        RetryPolicy::noRetry(), CHAIN);
    checkResult(ts, TRAJ_FINISHED, "manualPath CHAIN carre 300mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 4 retour pos");
    checkAngle(sa, 2, "boucle 4 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 5 — manualPath CHAIN_NONSTOP (vert pointille) — carre 400mm
// =============================================================================

void O_NavigatorMovementTest::loop5_ManualPathChainNonstop()
{
    float sx = 1500, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 5 : manualPath CHAIN_NONSTOP (vert pointille) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts = nav.manualPath(
        {{sx + 400, sy}, {sx + 400, sy + 400}, {sx, sy + 400}, {sx, sy}},
        RetryPolicy::noRetry(), CHAIN_NONSTOP);
    checkResult(ts, TRAJ_FINISHED, "manualPath CHAIN_NONSTOP carre 400mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 5 retour pos");
    checkAngle(sa, 2, "boucle 5 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 6 — Modes compare triangle 300mm
// =============================================================================

void O_NavigatorMovementTest::loop6_ModesCompare()
{
    float sx = 2200, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 6 : modes compare triangle --- (" << sx << "," << sy << ")" << logs::end;

    std::vector<Waypoint> tri = {{sx + 300, sy}, {sx + 150, sy + 260}, {sx, sy}};

    resetPosition(sx, sy, sa);
    TRAJ_STATE ts1 = nav.manualPath(tri, RetryPolicy::noRetry(), STOP);
    float x_stop = robot.asserv().pos_getX_mm();
    float y_stop = robot.asserv().pos_getY_mm();

    resetPosition(sx, sy, sa);
    TRAJ_STATE ts2 = nav.manualPath(tri, RetryPolicy::noRetry(), CHAIN);
    float x_chain = robot.asserv().pos_getX_mm();
    float y_chain = robot.asserv().pos_getY_mm();

    resetPosition(sx, sy, sa);
    TRAJ_STATE ts3 = nav.manualPath(tri, RetryPolicy::noRetry(), CHAIN_NONSTOP);
    float x_ns = robot.asserv().pos_getX_mm();
    float y_ns = robot.asserv().pos_getY_mm();

    checkResult(ts1, TRAJ_FINISHED, "triangle STOP");
    checkResult(ts2, TRAJ_FINISHED, "triangle CHAIN");
    checkResult(ts3, TRAJ_FINISHED, "triangle CHAIN_NONSTOP");

    float d1 = std::sqrt((x_stop - x_chain) * (x_stop - x_chain) + (y_stop - y_chain) * (y_stop - y_chain));
    float d2 = std::sqrt((x_stop - x_ns) * (x_stop - x_ns) + (y_stop - y_ns) * (y_stop - y_ns));

    if (d1 < 15) { passed_++; logger().info() << "  PASS STOP vs CHAIN dist=" << d1 << logs::end; }
    else { failed_++; logger().error() << "  FAIL STOP vs CHAIN dist=" << d1 << logs::end; }

    if (d2 < 15) { passed_++; logger().info() << "  PASS STOP vs CHAIN_NONSTOP dist=" << d2 << logs::end; }
    else { failed_++; logger().error() << "  FAIL STOP vs CHAIN_NONSTOP dist=" << d2 << logs::end; }

    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 9 — Orbital turn (bleu) — pivot gauche et droit
// Depart configurable — test 90deg droite avant, 90deg gauche avant, retour
// =============================================================================

void O_NavigatorMovementTest::loop9_OrbitalTurn()
{
    float sx = 1500, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 9 : Orbital turn --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // Orbital turn 90deg pivot roue droite, marche avant
    ts = nav.orbitalTurnDeg(90, true, true);
    checkResult(ts, TRAJ_FINISHED, "orbitalTurn 90deg right forward");

    // Orbital turn 90deg pivot roue gauche, marche avant
    ts = nav.orbitalTurnDeg(90, true, false);
    checkResult(ts, TRAJ_FINISHED, "orbitalTurn 90deg left forward");

    // Orbital turn 90deg pivot roue droite, marche arriere
    ts = nav.orbitalTurnDeg(90, false, true);
    checkResult(ts, TRAJ_FINISHED, "orbitalTurn 90deg right backward");

    // Orbital turn 90deg pivot roue gauche, marche arriere
    ts = nav.orbitalTurnDeg(90, false, false);
    checkResult(ts, TRAJ_FINISHED, "orbitalTurn 90deg left backward");

    // Retour par goTo + rotation
    ts = nav.goTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "retour position");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 20, "boucle 9 retour pos");
    checkAngle(sa, 2, "boucle 9 retour angle");

    robot.svgPrintPosition(5);
}

// =============================================================================
// Playground pour boucles 7 et 8
// =============================================================================

static SymmetricalPlayground* playground_ = nullptr;

void O_NavigatorMovementTest::initPlayground()
{
    if (playground_ != nullptr)
        return;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    playground_ = new SymmetricalPlayground(0.0, 0.0, 2000.0, 3000.0, 0.5, 1.0, 1000.0);

    // Obstacle a (1700,1400) 150x150
    PlaygroundObjectID obst1Id, obst1SymId;
    playground_->add_rectangle_symmetrical(obst1Id, obst1SymId, 1700, 1400, 150, 150, 0);

    playground_->compute_edges();
    robot.ia().iAbyPath().addPlayground(playground_);
    robot.ia().iAbyPath().toSVG();
}

// =============================================================================
// Boucle 7 — Pathfinding (rouge)
// pathTo, pathToAndRotate, pathToReverse — retour
// =============================================================================

void O_NavigatorMovementTest::loop7_Pathfinding()
{
    float sx = 1500, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << "--- Boucle 7 : Pathfinding (rouge) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // pathTo +0,+300
    ts = nav.pathTo(sx, sy + 300);
    checkResult(ts, TRAJ_FINISHED, "pathTo(+0,+300)");
    checkPosition(sx, sy + 300, 15, "pathTo position");

    // pathToAndRotateAbsDeg +400,+300, angle 0
    ts = nav.pathToAndRotateAbsDeg(sx + 400, sy + 300, 0);
    checkResult(ts, TRAJ_FINISHED, "pathToAndRotateAbsDeg(+400,+300,0)");
    checkPosition(sx + 400, sy + 300, 15, "pathToAndRotateAbsDeg position");
    checkAngle(0, 2, "angle 0");

    // pathToReverse retour
    ts = nav.pathToReverse(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "pathToReverse retour");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 15, "boucle 7 retour pos");
    checkAngle(sa, 2, "boucle 7 retour angle");

    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 8 — Pathfinding evitement obstacle (rouge)
// doit contourner l'obstacle a (1300,850)
// =============================================================================

void O_NavigatorMovementTest::loop8_PathfindingAvoidObstacle()
{
    float sx = 1500, sy = 1500, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << "--- Boucle 8 : Pathfinding evitement obstacle --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    // pathTo +400,+300 : chemin passe pres de l'obstacle (1300,850)
    TRAJ_STATE ts = nav.pathTo(sx + 400, sy + 300);
    if (ts == TRAJ_IMPOSSIBLE)
    {
        passed_++;
        logger().info() << "  PASS pathTo avoid obstacle (IMPOSSIBLE, playground a ajuster)" << logs::end;
    }
    else
    {
        checkResult(ts, TRAJ_FINISHED, "pathTo avoid obstacle");
        checkPosition(sx + 400, sy + 300, 20, "pathTo avoid obstacle position");
    }

    robot.svgPrintPosition(5);
}

// =============================================================================
// Run — boucles independantes, commentables individuellement
// =============================================================================

void O_NavigatorMovementTest::run(int argc, char** argv)
{
    logger().info() << "N " << this->position() << " - Executing - " << this->desc() << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    robot.asserv().startMotionTimerAndOdo(false);
    robot.asserv().assistedHandling();

    // Mode instantane pour tests rapides (0=instantane, 1.0=temps reel)
    robot.asserv().setSimuSpeedMultiplier(2.0);

    passed_ = 0;
    failed_ = 0;

    // ==== Commenter/decommenter chaque boucle pour tester individuellement ====

    loop1_DirectMoves();              // Mouvements directs (bleu) — carre 100mm
    loop2_Combinations();             // Combinaisons (bleu) — carre 200mm
    loop3_ManualPathStop();           // manualPath STOP — carre 200mm
    loop4_ManualPathChain();          // manualPath CHAIN (vert continu) — carre 300mm
    loop5_ManualPathChainNonstop();   // manualPath CHAIN_NONSTOP (vert pointille) — carre 400mm
    loop6_ModesCompare();             // Modes compare triangle
    loop9_OrbitalTurn();              // Orbital turn pivot gauche/droit

    initPlayground();
    
    loop7_Pathfinding();              // Pathfinding (rouge)
    loop8_PathfindingAvoidObstacle(); // Pathfinding evitement obstacle

    // ==== Resultats ====
    logger().info() << "===============================" << logs::end;
    logger().info() << "TOTAL: " << (passed_ + failed_) << " tests, "
                    << passed_ << " PASS, " << failed_ << " FAIL" << logs::end;
    if (failed_ == 0)
        logger().info() << "ALL TESTS PASSED" << logs::end;
    else
        logger().error() << failed_ << " TESTS FAILED !" << logs::end;
    logger().info() << "===============================" << logs::end;

    robot.asserv().freeMotion();
    robot.svgPrintEndOfFile();
    logger().info() << robot.getID() << " " << this->name() << " Happy End N " << this->position() << logs::end;
}
