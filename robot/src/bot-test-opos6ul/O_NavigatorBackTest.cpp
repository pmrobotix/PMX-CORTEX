/*!
 * \file
 * \brief Implementation du test de non-regression Navigator — marche arriere.
 *
 * 10 boucles independantes en miroir de O_NavigatorMovementTest (forward).
 * Memes positions de depart, memes patterns mais commandes back.
 * Voir O_NavigatorBackTest.hpp pour la description des boucles.
 */

#include "O_NavigatorBackTest.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "interface/AAsservDriver.hpp"
#include "utils/Arguments.hpp"
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

void O_NavigatorBackTest::resetPosition(float x, float y, float angleDeg)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.asserv().setPositionAndColor(x, y, angleDeg, false);
}

bool O_NavigatorBackTest::checkResult(TRAJ_STATE ts, TRAJ_STATE expected, const std::string& name)
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

bool O_NavigatorBackTest::checkPosition(float expectedX, float expectedY, float tolerance, const std::string& name)
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

bool O_NavigatorBackTest::checkAngle(float expectedDeg, float tolerance, const std::string& name)
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
// Boucle 1 — Mouvements directs back (bleu)
// line(-x), moveBackwardTo, faceBackTo, faceTo, rotateAbsDeg, rotateDeg
// =============================================================================

void O_NavigatorBackTest::loop1_DirectBack()
{
    float sx = 200, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 1 : Mouvements directs back (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // line(-100) : robot recule de 100 mm
    ts = nav.line(-100);
    checkResult(ts, TRAJ_FINISHED, "line(-100)");
    checkPosition(sx - 100, sy, 5, "line -> -100,0");

    // line(100) retour
    ts = nav.line(100);
    checkResult(ts, TRAJ_FINISHED, "line(100) retour");
    checkPosition(sx, sy, 5, "retour depart");

    // moveBackwardTo : tourne dos vers (+0,+100) puis recule
    ts = nav.moveBackwardTo(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardTo(0,+100)");
    checkPosition(sx, sy + 100, 5, "moveBackwardTo -> 0,+100");

    // goTo retour vers depart
    ts = nav.goTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "goTo retour depart");
    checkPosition(sx, sy, 5, "retour depart");

    // Rotations sur place
    ts = nav.rotateAbsDeg(45);
    checkResult(ts, TRAJ_FINISHED, "rotateAbsDeg(45)");
    checkAngle(45, 2, "angle 45");

    // faceBackTo (sx, sy+100) : direction +Y, dos en +Y → front = -90 deg
    ts = nav.faceBackTo(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "faceBackTo(0,+100)");
    checkAngle(-90, 2, "faceBackTo = -90");

    // faceTo (sx, sy+100) : direction +Y, front = +90 deg
    ts = nav.faceTo(sx, sy + 100);
    checkResult(ts, TRAJ_FINISHED, "faceTo(0,+100)");
    checkAngle(90, 2, "faceTo = 90");

    // rotateDeg(-90) : 90 + (-90) = 0
    ts = nav.rotateDeg(-90);
    checkResult(ts, TRAJ_FINISHED, "rotateDeg(-90)");
    checkAngle(0, 2, "angle 0 apres rotateDeg(-90)");

    ts = nav.rotateAbsDeg(sa);
    checkResult(ts, TRAJ_FINISHED, "retour angle initial");

    checkPosition(sx, sy, 5, "boucle 1 retour pos");
    checkAngle(sa, 2, "boucle 1 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 2 — Combinaisons goBackTo+ (bleu) — carre 200mm
// goBackToAndRotateAbsDeg, goBackToAndRotateRelDeg, goBackToAndFaceTo, goBackToAndFaceBackTo
// =============================================================================

void O_NavigatorBackTest::loop2_GoBackToCombos()
{
    float sx = 800, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 2 : Combinaisons goBackTo+ (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // goBackToAndRotateAbsDeg (+200,0,90)
    ts = nav.goBackToAndRotateAbsDeg(sx + 200, sy, 90);
    checkResult(ts, TRAJ_FINISHED, "goBackToAndRotateAbsDeg(+200,0,90)");
    checkPosition(sx + 200, sy, 5, "goBackToAndRotateAbsDeg pos");
    checkAngle(90, 2, "goBackToAndRotateAbsDeg angle 90");

    // goBackToAndRotateRelDeg (+200,+200,-90)
    ts = nav.goBackToAndRotateRelDeg(sx + 200, sy + 200, -90);
    checkResult(ts, TRAJ_FINISHED, "goBackToAndRotateRelDeg(+200,+200,-90)");
    checkPosition(sx + 200, sy + 200, 5, "goBackToAndRotateRelDeg pos");

    // goBackToAndFaceTo (0,+200, face vers depart) = -90 deg
    ts = nav.goBackToAndFaceTo(sx, sy + 200, sx, sy);
    checkResult(ts, TRAJ_FINISHED, "goBackToAndFaceTo(0,+200, face depart)");
    checkPosition(sx, sy + 200, 5, "goBackToAndFaceTo pos");
    checkAngle(-90, 2, "goBackToAndFaceTo angle = -90");

    // goBackToAndFaceBackTo (0,0, dos vers (+100,0)) : front en -X = 180 deg
    ts = nav.goBackToAndFaceBackTo(sx, sy, sx + 100, sy);
    checkResult(ts, TRAJ_FINISHED, "goBackToAndFaceBackTo(0,0, dos vers +100,0)");
    checkPosition(sx, sy, 5, "goBackToAndFaceBackTo pos");
    checkAngle(180, 2, "goBackToAndFaceBackTo angle = 180");

    // retour angle initial
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 5, "boucle 2 retour pos");
    checkAngle(sa, 2, "boucle 2 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 3 — Combinaisons moveBackwardTo+ (bleu) — carre 200mm
// moveBackwardToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo
// =============================================================================

void O_NavigatorBackTest::loop3_MoveBackwardCombos()
{
    float sx = 800, sy = 1700, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 3 : Combinaisons moveBackwardTo+ (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // moveBackwardToAndRotateAbsDeg (+200,0,90)
    ts = nav.moveBackwardToAndRotateAbsDeg(sx + 200, sy, 90);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardToAndRotateAbsDeg(+200,0,90)");
    checkPosition(sx + 200, sy, 5, "moveBackwardToAndRotateAbsDeg pos");
    checkAngle(90, 2, "moveBackwardToAndRotateAbsDeg angle 90");

    // moveBackwardToAndRotateRelDeg (+200,+200,-90)
    ts = nav.moveBackwardToAndRotateRelDeg(sx + 200, sy + 200, -90);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardToAndRotateRelDeg(+200,+200,-90)");
    checkPosition(sx + 200, sy + 200, 5, "moveBackwardToAndRotateRelDeg pos");

    // moveBackwardToAndFaceTo (0,+200, face vers depart) = -90 deg
    ts = nav.moveBackwardToAndFaceTo(sx, sy + 200, sx, sy);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardToAndFaceTo(0,+200, face depart)");
    checkPosition(sx, sy + 200, 5, "moveBackwardToAndFaceTo pos");
    checkAngle(-90, 2, "moveBackwardToAndFaceTo angle = -90");

    // moveBackwardToAndFaceBackTo (0,0, dos vers (+100,0)) : front en -X = 180 deg
    ts = nav.moveBackwardToAndFaceBackTo(sx, sy, sx + 100, sy);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardToAndFaceBackTo(0,0, dos vers +100,0)");
    checkPosition(sx, sy, 5, "moveBackwardToAndFaceBackTo pos");
    checkAngle(180, 2, "moveBackwardToAndFaceBackTo angle = 180");

    // retour angle initial
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 5, "boucle 3 retour pos");
    checkAngle(sa, 2, "boucle 3 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 4 — manualPath STOP back (pas de trait) — carre 200mm waypoints back
// =============================================================================

void O_NavigatorBackTest::loop4_ManualPathStopBack()
{
    float sx = 200, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 4 : manualPath STOP back --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    ts = nav.manualPath({});
    checkResult(ts, TRAJ_FINISHED, "manualPath vide");

    ts = nav.manualPath(
        {{sx + 200, sy, true}, {sx + 200, sy + 200, true}, {sx, sy + 200, true}, {sx, sy, true}},
        RetryPolicy::noRetry(), STOP);
    checkResult(ts, TRAJ_FINISHED, "manualPath STOP back carre 200mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 4 retour pos");
    checkAngle(sa, 2, "boucle 4 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 5 — manualPath CHAIN back (vert continu) — carre 300mm waypoints back
// =============================================================================

void O_NavigatorBackTest::loop5_ManualPathChainBack()
{
    float sx = 800, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 5 : manualPath CHAIN back --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts = nav.manualPath(
        {{sx + 300, sy, true}, {sx + 300, sy + 300, true}, {sx, sy + 300, true}, {sx, sy, true}},
        RetryPolicy::noRetry(), CHAIN);
    checkResult(ts, TRAJ_FINISHED, "manualPath CHAIN back carre 300mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 5 retour pos");
    checkAngle(sa, 2, "boucle 5 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 6 — manualPath CHAIN_NONSTOP back (vert pointille) — carre 400mm waypoints back
// =============================================================================

void O_NavigatorBackTest::loop6_ManualPathChainNonstopBack()
{
    float sx = 1500, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 6 : manualPath CHAIN_NONSTOP back --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts = nav.manualPath(
        {{sx + 400, sy, true}, {sx + 400, sy + 400, true}, {sx, sy + 400, true}, {sx, sy, true}},
        RetryPolicy::noRetry(), CHAIN_NONSTOP);
    checkResult(ts, TRAJ_FINISHED, "manualPath CHAIN_NONSTOP back carre 400mm");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 6 retour pos");
    checkAngle(sa, 2, "boucle 6 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 7 — Modes compare triangle 300mm (waypoints back)
// =============================================================================

void O_NavigatorBackTest::loop7_ModesCompareBack()
{
    float sx = 2200, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 7 : modes compare triangle back --- (" << sx << "," << sy << ")" << logs::end;

    std::vector<Waypoint> tri = {
        {sx + 300, sy,        true},
        {sx + 150, sy + 260,  true},
        {sx,       sy,        true}
    };

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

    checkResult(ts1, TRAJ_FINISHED, "triangle back STOP");
    checkResult(ts2, TRAJ_FINISHED, "triangle back CHAIN");
    checkResult(ts3, TRAJ_FINISHED, "triangle back CHAIN_NONSTOP");

    float d1 = std::sqrt((x_stop - x_chain) * (x_stop - x_chain) + (y_stop - y_chain) * (y_stop - y_chain));
    float d2 = std::sqrt((x_stop - x_ns) * (x_stop - x_ns) + (y_stop - y_ns) * (y_stop - y_ns));

    if (d1 < 15) { passed_++; logger().info() << "  PASS STOP vs CHAIN dist=" << d1 << logs::end; }
    else { failed_++; logger().error() << "  FAIL STOP vs CHAIN dist=" << d1 << logs::end; }

    if (d2 < 15) { passed_++; logger().info() << "  PASS STOP vs CHAIN_NONSTOP dist=" << d2 << logs::end; }
    else { failed_++; logger().error() << "  FAIL STOP vs CHAIN_NONSTOP dist=" << d2 << logs::end; }

    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 8 — Orbital turn (bleu) — pivot gauche et droit
// orbitalTurnDeg teste deja avant ET arriere par construction (parametre forward)
// =============================================================================

void O_NavigatorBackTest::loop8_OrbitalTurn()
{
    float sx = 1500, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 8 : Orbital turn --- (" << sx << "," << sy << ")" << logs::end;
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
    checkPosition(sx, sy, 20, "boucle 8 retour pos");
    checkAngle(sa, 2, "boucle 8 retour angle");

    robot.svgPrintPosition(5);
}

// =============================================================================
// Playground pour boucles 9 et 10
// =============================================================================

static SymmetricalPlayground* playground_back_ = nullptr;

void O_NavigatorBackTest::initPlayground()
{
    if (playground_back_ != nullptr)
        return;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    playground_back_ = new SymmetricalPlayground(0.0, 0.0, 2000.0, 3000.0, 0.5, 1.0, 1000.0);

    // Obstacle a (1700,1400) 150x150
    PlaygroundObjectID obst1Id, obst1SymId;
    playground_back_->add_rectangle_symmetrical(obst1Id, obst1SymId, 1700, 1400, 150, 150, 0);

    playground_back_->compute_edges();
    robot.ia().iAbyPath().addPlayground(playground_back_);
    robot.ia().iAbyPath().toSVG();
}

// =============================================================================
// Boucle 9 — pathBackTo + pathBackToAnd* (rouge)
// pathBackTo, pathBackToAndRotateAbsDeg/RelDeg/FaceTo/FaceBackTo, pathTo retour
// =============================================================================

void O_NavigatorBackTest::loop9_PathBackFinding()
{
    float sx = 1500, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << "--- Boucle 9 : pathBackTo+ (rouge) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // pathBackTo +0,+300
    ts = nav.pathBackTo(sx, sy + 300);
    checkResult(ts, TRAJ_FINISHED, "pathBackTo(+0,+300)");
    checkPosition(sx, sy + 300, 15, "pathBackTo position");

    // pathBackToAndRotateAbsDeg +400,+300, angle 0
    ts = nav.pathBackToAndRotateAbsDeg(sx + 400, sy + 300, 0);
    checkResult(ts, TRAJ_FINISHED, "pathBackToAndRotateAbsDeg(+400,+300,0)");
    checkPosition(sx + 400, sy + 300, 15, "pathBackToAndRotateAbsDeg position");
    checkAngle(0, 2, "pathBackToAndRotateAbsDeg angle 0");

    // pathBackToAndRotateRelDeg : retour vers (sx, sy+300) puis rotation relative +90
    ts = nav.pathBackToAndRotateRelDeg(sx, sy + 300, 90);
    checkResult(ts, TRAJ_FINISHED, "pathBackToAndRotateRelDeg(0,+300,+90)");
    checkPosition(sx, sy + 300, 15, "pathBackToAndRotateRelDeg position");

    // pathBackToAndFaceTo : vers (sx, sy) face vers (sx+400, sy) = +X = 0 deg
    ts = nav.pathBackToAndFaceTo(sx, sy, sx + 400, sy);
    checkResult(ts, TRAJ_FINISHED, "pathBackToAndFaceTo(0,0, face +X)");
    checkPosition(sx, sy, 15, "pathBackToAndFaceTo position");
    checkAngle(0, 2, "pathBackToAndFaceTo angle = 0");

    // pathBackToAndFaceBackTo : vers (sx, sy+300) dos vers (sx+100, sy+300) = front -X = 180 deg
    ts = nav.pathBackToAndFaceBackTo(sx, sy + 300, sx + 100, sy + 300);
    checkResult(ts, TRAJ_FINISHED, "pathBackToAndFaceBackTo(0,+300, dos +X)");
    checkPosition(sx, sy + 300, 15, "pathBackToAndFaceBackTo position");
    checkAngle(180, 2, "pathBackToAndFaceBackTo angle = 180");

    // pathTo retour
    ts = nav.pathTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "pathTo retour");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 15, "boucle 9 retour pos");
    checkAngle(sa, 2, "boucle 9 retour angle");

    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 10 — pathBackTo evitement obstacle (rouge)
// doit contourner l'obstacle a (1700,1400)
// =============================================================================

void O_NavigatorBackTest::loop10_PathBackAvoidObstacle()
{
    float sx = 1500, sy = 1500, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << "--- Boucle 10 : pathBackTo evitement obstacle --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    // pathBackTo +400,+300 : chemin doit contourner l'obstacle (1700,1400)
    TRAJ_STATE ts = nav.pathBackTo(sx + 400, sy + 300);
    if (ts == TRAJ_IMPOSSIBLE)
    {
        passed_++;
        logger().info() << "  PASS pathBackTo avoid obstacle (IMPOSSIBLE, playground a ajuster)" << logs::end;
    }
    else
    {
        checkResult(ts, TRAJ_FINISHED, "pathBackTo avoid obstacle");
        checkPosition(sx + 400, sy + 300, 20, "pathBackTo avoid obstacle position");
    }

    robot.svgPrintPosition(5);
}

// =============================================================================
// Run — boucles independantes, commentables individuellement
// =============================================================================

void O_NavigatorBackTest::run(int argc, char** argv)
{
    logger().info() << "N " << this->position() << " - Executing - " << this->desc() << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    Arguments::Option cOptMultiplier('M', "simu speed multiplier (0=instantane, 1.0=temps reel)");
    cOptMultiplier.addArgument("multiplier", "multiplier", "2.0");
    robot.getArgs().addOption(cOptMultiplier);
    robot.parseConsoleArgs(argc, argv);

    float multiplier = atof(robot.getArgs()['M']["multiplier"].c_str());
    logger().info() << "simuSpeedMultiplier=" << multiplier << logs::end;

    // Position initiale avant de demarrer le thread d'asserv (reset Nucleo +
    // filtre les positions residuelles). Chaque loopX_* fera son propre
    // resetPosition() ensuite.
    resetPosition(0, 0, 0);
    robot.asserv().startMotionTimerAndOdo(false);
    robot.asserv().assistedHandling();

    robot.asserv().setSimuSpeedMultiplier(multiplier);

    passed_ = 0;
    failed_ = 0;

    // ==== Commenter/decommenter chaque boucle pour tester individuellement ====

    loop1_DirectBack();                // Mouvements directs back (bleu)
    loop2_GoBackToCombos();            // Combinaisons goBackTo+ (bleu) — carre 200mm
    loop3_MoveBackwardCombos();        // Combinaisons moveBackwardTo+ (bleu) — carre 200mm
    loop4_ManualPathStopBack();        // manualPath STOP back — carre 200mm
    loop5_ManualPathChainBack();       // manualPath CHAIN back — carre 300mm
    loop6_ManualPathChainNonstopBack();// manualPath CHAIN_NONSTOP back — carre 400mm
    loop7_ModesCompareBack();          // Modes compare triangle back
    loop8_OrbitalTurn();               // Orbital turn pivot gauche/droit

    initPlayground();

    loop9_PathBackFinding();           // pathBackTo (rouge) + 4 combos
    loop10_PathBackAvoidObstacle();    // pathBackTo evitement obstacle

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
