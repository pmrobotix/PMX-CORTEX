/*!
 * \file
 * \brief Implementation du test de non-regression Navigator — marche arriere.
 *
 * Boucles independantes, chacune revient a son point de depart.
 * Chaque boucle a un point de depart configurable (sx, sy).
 * Tous les waypoints sont en offset relatif par rapport a ce depart.
 */

#include "O_NavigatorBackTest.hpp"

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
// Boucle 1 — Mouvements directs arriere (bleu)
// goBackTo, faceBackTo, moveBackwardTo — retour au depart
// =============================================================================

void O_NavigatorBackTest::loop1_DirectBack()
{
    float sx = 400, sy = 300, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 1 : Mouvements directs arriere (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // goBackTo (+100, 0) : robot tourne le dos vers (+100,0) puis recule
    ts = nav.goBackTo(sx + 100, sy);
    checkResult(ts, TRAJ_FINISHED, "goBackTo(+100,0)");
    checkPosition(sx + 100, sy, 5, "goBackTo -> +100,0");

    // faceBackTo (0, 0) = dos vers depart → angle 0 (depart est derriere)
    ts = nav.faceBackTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "faceBackTo(depart)");
    checkAngle(0, 2, "faceBackTo = 0 deg");

    // moveBackwardTo (0, 0) : rotation dos au point + ligne arriere vers depart
    ts = nav.moveBackwardTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "moveBackwardTo(depart)");
    checkPosition(sx, sy, 5, "moveBackwardTo -> depart");

    // retour angle initial
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 5, "boucle 1 retour pos");
    checkAngle(sa, 2, "boucle 1 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 2 — Combinaisons avec FaceBackTo (bleu) — carre 200mm
// goToAndFaceBackTo, moveForwardToAndFaceBackTo — retour
// =============================================================================

void O_NavigatorBackTest::loop2_CombinationsBack()
{
    float sx = 400, sy = 700, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 2 : Combinaisons FaceBackTo (bleu) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // goToAndFaceBackTo (+200, 0, dos vers depart) : arrive en (+200,0), tourne dos vers (0,0)
    // dos vers (sx,sy) depuis (sx+200,sy) = face +X = angle 0 → dos = 180... non
    // depuis (+200,0) vers depart (0,0) : direction = -X (180 deg), donc face = 180, dos = 0
    ts = nav.goToAndFaceBackTo(sx + 200, sy, sx, sy);
    checkResult(ts, TRAJ_FINISHED, "goToAndFaceBackTo(+200,0, dos vers depart)");
    checkPosition(sx + 200, sy, 5, "(+200,0)");
    checkAngle(0, 2, "dos vers depart = 0 deg");

    // moveForwardToAndFaceBackTo (+200,+200, dos vers +400,+200)
    // depuis (+200,+200) vers (+400,+200) : direction = +X (0 deg), dos = 180 deg
    ts = nav.moveForwardToAndFaceBackTo(sx + 200, sy + 200, sx + 400, sy + 200);
    checkResult(ts, TRAJ_FINISHED, "moveForwardToAndFaceBackTo(+200,+200, dos vers +400,+200)");
    checkPosition(sx + 200, sy + 200, 5, "(+200,+200)");
    checkAngle(180, 2, "dos vers +X = 180 deg");

    // retour depart
    ts = nav.goTo(sx, sy);
    checkResult(ts, TRAJ_FINISHED, "retour depart");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 5, "boucle 2 retour pos");
    checkAngle(sa, 2, "boucle 2 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 3 — manualPath mixte (waypoints avant + arriere) — carre 300mm
// =============================================================================

void O_NavigatorBackTest::loop3_ManualPathMixed()
{
    float sx = 400, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 3 : manualPath mixte --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    // Carre : avant -> avant -> arriere -> arriere (retour)
    TRAJ_STATE ts = nav.manualPath(
        {
            {sx + 300, sy,        false},  // avant
            {sx + 300, sy + 300,  false},  // avant
            {sx,       sy + 300,  true},   // arriere
            {sx,       sy,        true}    // arriere
        },
        RetryPolicy::noRetry(), STOP);
    checkResult(ts, TRAJ_FINISHED, "manualPath mixte STOP");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 3 retour pos");
    checkAngle(sa, 2, "boucle 3 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Boucle 4 — manualPath tout arriere (CHAIN) — carre 300mm
// =============================================================================

void O_NavigatorBackTest::loop4_ManualPathAllBack()
{
    float sx = 1000, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot);

    logger().info() << "--- Boucle 4 : manualPath tout arriere (CHAIN) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts = nav.manualPath(
        {
            {sx + 300, sy,        true},
            {sx + 300, sy + 300,  true},
            {sx,       sy + 300,  true},
            {sx,       sy,        true}
        },
        RetryPolicy::noRetry(), CHAIN);
    checkResult(ts, TRAJ_FINISHED, "manualPath tout arriere CHAIN");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 10, "boucle 4 retour pos");
    checkAngle(sa, 2, "boucle 4 retour angle");
    robot.svgPrintPosition(5);
}

// =============================================================================
// Playground pour boucle 5
// =============================================================================

static SymmetricalPlayground* playground_back_ = nullptr;

void O_NavigatorBackTest::initPlayground()
{
    if (playground_back_ != nullptr)
        return;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    playground_back_ = new SymmetricalPlayground(0.0, 0.0, 2000.0, 3000.0, 0.5, 1.0, 1000.0);
    playground_back_->compute_edges();
    robot.ia().iAbyPath().addPlayground(playground_back_);
    robot.ia().iAbyPath().toSVG();
}

// =============================================================================
// Boucle 5 — pathBackTo dans les 3 modes (rouge)
// STOP, CHAIN, CHAIN_NONSTOP — retour
// =============================================================================

void O_NavigatorBackTest::loop5_PathBackModes()
{
    float sx = 1700, sy = 1200, sa = 0; // <-- depart configurable (x, y, angle deg)

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << "--- Boucle 5 : pathBackTo 3 modes (rouge) --- (" << sx << "," << sy << ")" << logs::end;
    resetPosition(sx, sy, sa);
    robot.svgPrintPosition();

    TRAJ_STATE ts;

    // pathBackTo STOP vers (+300, 0)
    ts = nav.pathBackTo(sx + 300, sy, RetryPolicy::noRetry(), STOP);
    checkResult(ts, TRAJ_FINISHED, "pathBackTo STOP (+300,0)");
    checkPosition(sx + 300, sy, 15, "pathBackTo STOP position");

    // pathBackTo CHAIN vers (+300, +300)
    ts = nav.pathBackTo(sx + 300, sy + 300, RetryPolicy::noRetry(), CHAIN);
    checkResult(ts, TRAJ_FINISHED, "pathBackTo CHAIN (+300,+300)");
    checkPosition(sx + 300, sy + 300, 15, "pathBackTo CHAIN position");

    // pathBackTo CHAIN_NONSTOP retour (0, 0)
    ts = nav.pathBackTo(sx, sy, RetryPolicy::noRetry(), CHAIN_NONSTOP);
    checkResult(ts, TRAJ_FINISHED, "pathBackTo CHAIN_NONSTOP retour");
    nav.rotateAbsDeg(sa);
    checkPosition(sx, sy, 15, "boucle 5 retour pos");
    checkAngle(sa, 2, "boucle 5 retour angle");

    robot.svgPrintPosition(5);
}

// =============================================================================
// Run — boucles independantes, commentables individuellement
// =============================================================================

void O_NavigatorBackTest::run(int argc, char** argv)
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

    loop1_DirectBack();          // goBackTo, faceBackTo, moveBackwardTo
    loop2_CombinationsBack();    // goToAndFaceBackTo, moveForwardToAndFaceBackTo
    loop3_ManualPathMixed();     // manualPath mixte avant/arriere
    loop4_ManualPathAllBack();   // manualPath tout arriere CHAIN

    initPlayground();
    loop5_PathBackModes();       // pathBackTo 3 modes

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
