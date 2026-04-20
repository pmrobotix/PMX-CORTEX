/*!
 * \file
 * \brief Test de navigation en carre via Navigator::moveForwardTo().
 *
 * Part du coin (x, y, a), trace un carre de cote d en marche avant,
 * avec rotation absolue a chaque coin, revient au point de depart et
 * oriente selon l'angle a. Repete nb fois. Peut tourner en sens
 * anti-horaire (CCW, defaut) ou horaire (CW).
 *
 * La couleur (miroir x) suit la couleur choisie au menu : color0=bleu
 * -> pas de miroir, color3000=jaune -> x mirrore (convention PMX
 * single-color, cf STRATEGY_JSON_FORMAT.md).
 *
 * Arguments positionnels : x y a d nb cw
 *   x   : x du coin de depart mm (defaut 300)
 *   y   : y du coin de depart mm (defaut 300)
 *   a   : angle de depart degre (defaut 0)
 *   d   : cote du carre mm      (defaut 500)
 *   nb  : nombre de tours       (defaut 1)
 *   cw  : 0=CCW (defaut), 1=CW
 *
 * Couleur (convention PMX color0=bleu primaire) :
 *   - defaut    : BLEU (pas de miroir, coords litterales)
 *   - /y        : JAUNE (miroir x -> 3000-x applique)
 *
 * === SIMU ===
 *
 *   ./bot-opos6ul sq                        # defauts, BLEU
 *   ./bot-opos6ul sq 400 300 0 500 2        # 2 tours CCW cote BLEU
 *   ./bot-opos6ul sq 400 300 0 500 1 1      # 1 tour CW cote BLEU
 *   ./bot-opos6ul /y sq 400 300 0 500 2     # 2 tours CCW cote JAUNE (miroir)
 *   ./bot-opos6ul sq 500 500 0 1000 1       # grand carre 1m BLEU
 *
 * === Vrai robot (ARM) ===
 *
 *   ./bot-opos6ul sq 300 300 0 400 1        # petit carre 400mm BLEU, prudent
 *   ./bot-opos6ul sq 500 500 0 600 3        # 3 tours pour repetabilite
 *   ./bot-opos6ul /y sq 500 500 0 600 1 1   # sens horaire cote JAUNE
 */

#include "O_Asserv_SquareTest.hpp"

#include <cstdlib>
#include <string>

#include "action/Sensors.hpp"
#include "utils/Arguments.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_Asserv_SquareTest::configureConsoleArgs(int argc, char** argv)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    // Position de depart = 1er coin du carre (pas d'offset initial).
    // Defaut (300, 300, 0 deg) pour etre a l'interieur de la table (robot r=140).
    robot.getArgs().addArgument("x",  "x start (1er coin) mm", "300");
    robot.getArgs().addArgument("y",  "y start (1er coin) mm", "300");
    robot.getArgs().addArgument("a",  "angle start (deg)",     "0");
    robot.getArgs().addArgument("d",  "cote du carre mm",      "500");
    robot.getArgs().addArgument("nb", "nombre de tours",       "1");
    robot.getArgs().addArgument("cw", "sens horaire (0=CCW defaut, 1=CW)", "0");

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

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Arguments args = robot.getArgs();

    float x  = atof(args["x"].c_str());
    float y  = atof(args["y"].c_str());
    float a  = atof(args["a"].c_str());
    float d  = atof(args["d"].c_str());
    int   nb = atoi(args["nb"].c_str());
    bool  cw = (atoi(args["cw"].c_str()) != 0);

    logger().info() << "args x=" << x << " y=" << y << " a=" << a
                    << " d=" << d << " nb=" << nb
                    << " sens=" << (cw ? "CW" : "CCW") << logs::end;

    // setPositionAndColor AVANT startMotionTimerAndOdo.
    // Couleur respectee depuis le menu / CLI (/b = BLUE, sinon YELLOW par defaut).
    // Convention PMX : coords (x, y) ecrites en repere BLEU (color0). Si robot
    // est JAUNE, matchColor=true applique le miroir x -> 3000-x automatiquement.
    // Pour lancer cote bleu : passer /b en ligne de commande.
    bool isYellow = (robot.getMyColor() == PMXYELLOW);
    logger().info() << "myColor=" << (isYellow ? "YELLOW (miroir x)" : "BLUE (pas de miroir)")
                    << logs::end;
    robot.asserv().setPositionAndColor(x, y, a, isYellow);
    robot.asserv().startMotionTimerAndOdo(true);
    robot.asserv().assistedHandling();

    robot.svgPrintPosition();

    robot.actions().start();
    // Moves en marche avant uniquement -> front actif, back ignore.
    robot.actions().sensors().setIgnoreFrontNearObstacle(false, false, false);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
    robot.actions().sensors().startSensorsThread(20);

    chrono.start();

    Navigator nav(&robot);

    // Helper local : log pos + encodeurs + SVG.
    auto logStep = [&](const char* step) {
        robot.asserv().getEncodersCounts(&right, &left);
        ROBOTPOSITION p = robot.asserv().pos_getPosition();
        logger().info() << step
                        << " time=" << robot.chrono().getElapsedTimeInMilliSec() << "ms"
                        << " L=" << left << " R=" << right
                        << " pos=(" << p.x << "," << p.y << ","
                        << (p.theta * 180.0 / M_PI) << "deg)" << logs::end;
        robot.svgPrintPosition();
    };

    // Ordre des coins. CCW (defaut) : E -> N -> O -> S.
    //                  CW           : N -> E -> S -> O.
    // Le dernier coin est toujours le coin 1 (retour avec rotation finale).
    float c2x, c2y, c3x, c3y, c4x, c4y;
    if (!cw) {
        // Anti-horaire : coin2=(x+d, y), coin3=(x+d, y+d), coin4=(x, y+d)
        c2x = x + d; c2y = y;
        c3x = x + d; c3y = y + d;
        c4x = x;     c4y = y + d;
    } else {
        // Horaire : coin2=(x, y+d), coin3=(x+d, y+d), coin4=(x+d, y)
        c2x = x;     c2y = y + d;
        c3x = x + d; c3y = y + d;
        c4x = x + d; c4y = y;
    }

    for (int n = 1; n <= nb; n++)
    {
        logger().info() << "=== Tour " << n << "/" << nb
                        << " " << (cw ? "CW" : "CCW")
                        << " - carre a partir de ("
                        << x << "," << y << ") cote " << d << "mm ===" << logs::end;

        logger().info() << "moveForwardTo(" << c2x << ", " << c2y << ")" << logs::end;
        nav.moveForwardTo(c2x, c2y);
        logStep("coin2");

        logger().info() << "moveForwardTo(" << c3x << ", " << c3y << ")" << logs::end;
        nav.moveForwardTo(c3x, c3y);
        logStep("coin3");

        logger().info() << "moveForwardTo(" << c4x << ", " << c4y << ")" << logs::end;
        nav.moveForwardTo(c4x, c4y);
        logStep("coin4");

        // Retour coin1 + orientation finale (angle de depart).
        logger().info() << "moveForwardToAndRotateAbsDeg(" << x << ", " << y
                        << ", " << a << ")" << logs::end;
        nav.moveForwardToAndRotateAbsDeg(x, y, a);
        logStep("retour_coin1");
    }

    logger().info() << "=== End " << nb << " tour(s) - time="
                    << chrono.getElapsedTimeInMilliSec() << "ms"
                    << " pos_final=(" << robot.asserv().pos_getX_mm()
                    << "," << robot.asserv().pos_getY_mm()
                    << "," << robot.asserv().pos_getThetaInDegree()
                    << "deg) ===" << logs::end;

    logger().info() << robot.getID() << " " << this->name() << " Happy End"
                    << " N° " << this->position() << logs::end;
}
