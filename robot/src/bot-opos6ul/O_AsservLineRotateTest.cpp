/*!
 * \file
 * \brief Implementation de la classe O_AsservLineRotateTest (version Navigator).
 *
 * Voir O_AsservLineRotateTest.hpp pour les exemples complets.
 *
 * Options : /m 0=relatif(defaut) 1=absolu  /s vitesse%  /r repetitions
 *           /p 0=asserv direct 1=Navigator line  /B detection  /+ x y a
 *
 * === SIMU — carre 500mm ===
 *
 *   ./bot-opos6ul t /n 8 500 90 0  500 90 0  500 90 0  500 90 0 /+ 250 250 0
 *   ./bot-opos6ul t /n 8 500 90 0  500 90 0  500 90 0  500 90 0 /p 1 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   ./bot-opos6ul t /n 8 200 90 0  200 90 0  200 90 0  200 90 0 /s 20 /+ 100 100 0
 *   ./bot-opos6ul t /n 8 200 90 0  200 90 0  200 90 0  200 90 0 /p 1 /s 20 /+ 100 100 0
 */

#include "O_AsservLineRotateTest.hpp"

#include <cstdlib>
#include <string>

#include "action/Sensors.hpp"
#include "utils/Arguments.hpp"
#include "interface/AAsservDriver.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "navigator/Navigator.hpp"
#include "navigator/RetryPolicy.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_AsservLineRotateTest::configureConsoleArgs(int argc, char **argv)
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	robot.getArgs().addArgument("d", "distance mm");
	robot.getArgs().addArgument("a", "angle degres", "-1");
	robot.getArgs().addArgument("back", "backwards[0,1]", "0");

	robot.getArgs().addArgument("d2", "distance mm", "-1");
	robot.getArgs().addArgument("a2", "angle degres", "-1");
	robot.getArgs().addArgument("back2", "backwards[0,1]", "0");

	robot.getArgs().addArgument("d3", "distance mm", "-1");
	robot.getArgs().addArgument("a3", "angle degres", "-1");
	robot.getArgs().addArgument("back3", "backwards[0,1]", "0");

	robot.getArgs().addArgument("d4", "distance mm", "-1");
	robot.getArgs().addArgument("a4", "angle degres", "-1");
	robot.getArgs().addArgument("back4", "backwards[0,1]", "0");

	robot.getArgs().addArgument("d5", "distance mm", "-1");
	robot.getArgs().addArgument("a5", "angle degres", "-1");
	robot.getArgs().addArgument("back5", "backwards[0,1]", "0");

	//mode de drive
	Arguments::Option cOptMode('m', "mode used for test");
	cOptMode.addArgument("mode", "0:relative 1:absolute", "0");
	robot.getArgs().addOption(cOptMode);

	//mode de pathfinding
	Arguments::Option cOptPMode('p', "mode Navigator");
	cOptPMode.addArgument("pmode", "0:asserv direct 1:nav.line", "0");
	robot.getArgs().addOption(cOptPMode);

	//speed
	Arguments::Option cOptSpeed('s', "speed en %");
	cOptSpeed.addArgument("speed", "speed en %", "40");
	robot.getArgs().addOption(cOptSpeed);

	//detection adv
	Arguments::Option cOptdetect('B', "Detection Balise");
	cOptdetect.addArgument("detection", "[0-1]", "1");
	robot.getArgs().addOption(cOptdetect);

	//repetition du cycle
	Arguments::Option cOptRepeat('r', "nombre de repetitions du cycle");
	cOptRepeat.addArgument("repeat", "nombre de repetitions", "1");
	robot.getArgs().addOption(cOptRepeat);

	Arguments::Option cOpt('+', "Coordinates x,y,a");
	cOpt.addArgument("coordx", "coord x mm", "300.0");
	cOpt.addArgument("coordy", "coord y mm", "300.0");
	cOpt.addArgument("coorda", "coord teta deg", "0.0");
	robot.getArgs().addOption(cOpt);

	//reparse arguments
	robot.parseConsoleArgs(argc, argv);
}

void O_AsservLineRotateTest::run(int argc, char **argv)
{
	logger().info() << "N " << this->position() << " - Executing - " << this->desc() << logs::end;
	configureConsoleArgs(argc, argv);

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	Arguments args = robot.getArgs();

	// Parse les 5 segments (d, a, back)
	struct Segment { float d; float a; bool back; };
	Segment segments[5];
	const char* dNames[] = {"d", "d2", "d3", "d4", "d5"};
	const char* aNames[] = {"a", "a2", "a3", "a4", "a5"};
	const char* bNames[] = {"back", "back2", "back3", "back4", "back5"};

	for (int i = 0; i < 5; i++)
	{
		segments[i].d = atof(args[dNames[i]].c_str());
		segments[i].a = atof(args[aNames[i]].c_str());
		segments[i].back = atoi(args[bNames[i]].c_str());
	}

	int B = atoi(args['B']["detection"].c_str());
	int s = atoi(args['s']["speed"].c_str());
	int m = atoi(args['m']["mode"].c_str());
	int navMode = atoi(args['p']["pmode"].c_str());
	int repeat = atoi(args['r']["repeat"].c_str());

	float coordx = atof(args['+']["coordx"].c_str());
	float coordy = atof(args['+']["coordy"].c_str());
	float coorda_deg = atof(args['+']["coorda"].c_str());

	logger().info() << "navMode=" << navMode << " mode=" << m
	                << " speed=" << s << " repeat=" << repeat
	                << " detection=" << B << logs::end;
	logger().info() << "COORD cx=" << coordx << " cy=" << coordy
	                << " ca=" << coorda_deg << logs::end;

	// Init asserv : setPositionAndColor AVANT startMotionTimerAndOdo
	// (le set position reset la Nucleo et definit la couleur de match)
	robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, (bool)(robot.getMyColor() != PMXYELLOW));
	robot.asserv().startMotionTimerAndOdo(false);
	robot.asserv().assistedHandling();

	ROBOTPOSITION pos = robot.asserv().pos_getPosition();
	logger().info() << "time=" << robot.chrono().getElapsedTimeInMilliSec() << "ms"
	                << " px=" << pos.x << " py=" << pos.y
	                << " pa_deg=" << pos.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();

	// Init actions + detection
	robot.actions().start();
	if (B == 1)
	{
		robot.actions().sensors().addTimerSensors(20);
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	}
	else
	{
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	}
	robot.chrono().start();

	// Vitesse reduite
	robot.asserv().setMaxSpeed(true, s, s);

	// Navigator
	Navigator nav(&robot, &robot.ia().iAbyPath());

	// RetryPolicy pour le mode Navigator
	RetryPolicy policy = RetryPolicy::aggressive();
	policy.rotateIgnoringOpponent = (B == 0); // si pas de detection, on ignore opponent

	TRAJ_STATE ts = TRAJ_IDLE;

	for (int rep = 1; rep <= repeat; rep++)
	{
		logger().info() << "=== REPETITION " << rep << "/" << repeat << " ===" << logs::end;

		for (int nb = 0; nb < 5; nb++)
		{
			float dd = segments[nb].d;
			float aa = segments[nb].a;
			bool bback = segments[nb].back;

			if (dd == -1)
				continue; // segment non defini

			logger().info() << "=> TRAJET n " << (nb + 1)
			                << " d=" << dd << " a=" << aa << " back=" << bback << logs::end;

			if (bback)
				dd = -dd;

			// ========== Deplacement ==========
			if (navMode == 0)
			{
				// Mode 0 : asserv direct (pas de retry)
				ts = robot.asserv().line(dd);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " WHAT TO DO ?" << logs::end;
					robot.asserv().resetEmergencyOnTraj("line: " + robot.asserv().getTraj(ts));
				}
			}
			else if (navMode == 1)
			{
				// Mode 1 : Navigator line (retry)
				ts = nav.line(dd, policy);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " nav.line ECHEC FINAL" << logs::end;
				}
			}

			robot.svgPrintPosition();

			// ========== Rotation ==========
			if (aa != -1)
			{
				if (m == 0)
				{
					// Rotation relative
					if (navMode >= 1)
					{
						ts = nav.rotateDeg(aa, policy);
					}
					else
					{
						ts = nav.rotateDeg(aa, RetryPolicy::noRetry());
						if (ts >= TRAJ_INTERRUPTED)
						{
							logger().info() << robot.asserv().getTraj(ts) << " rotateDeg ECHEC" << logs::end;
							robot.asserv().resetEmergencyOnTraj("rotateDeg: " + robot.asserv().getTraj(ts));
						}
					}
				}
				else if (m == 1)
				{
					// Rotation absolue
					if (navMode >= 1)
					{
						ts = nav.rotateAbsDeg(aa, policy);
					}
					else
					{
						ts = nav.rotateAbsDeg(aa, RetryPolicy::noRetry());
						if (ts >= TRAJ_INTERRUPTED)
						{
							logger().info() << robot.asserv().getTraj(ts) << " rotateAbsDeg ECHEC" << logs::end;
							robot.asserv().resetEmergencyOnTraj("rotateAbsDeg: " + robot.asserv().getTraj(ts));
						}
					}
				}
				robot.svgPrintPosition();
			}
		}
	} // fin repetition

	robot.svgPrintPosition();
	robot.asserv().freeMotion();

	pos = robot.asserv().pos_getPosition();
	logger().info() << "time=" << robot.chrono().getElapsedTimeInMilliSec() << "ms"
	                << " x=" << pos.x << " y=" << pos.y
	                << " deg=" << pos.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();
	robot.svgPrintEndOfFile();
	logger().info() << robot.getID() << " " << this->name() << " Happy End N " << this->position() << logs::end;
}
