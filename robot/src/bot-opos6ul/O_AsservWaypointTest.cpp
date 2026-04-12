/*!
 * \file
 * \brief Implementation de la classe O_AsservWaypointTest.
 *
 * Test de navigation par waypoints via Navigator::manualPath().
 * Jusqu'a 5 waypoints (x,y) avec choix du PathMode.
 *
 * Options : /m 0=STOP 1=CHAIN 2=CHAIN_NONSTOP
 *           /s vitesse%  /B detection  /+ x y a (position initiale)
 *
 * Voir O_AsservWaypointTest.hpp pour les exemples complets.
 *
 * === SIMU — carre 500mm (depart 250,250) ===
 *
 *   ./bot-opos6ul t /n 9 750 250  750 750  250 750  250 250 /m 0 /+ 250 250 0
 *   ./bot-opos6ul t /n 9 750 250  750 750  250 750  250 250 /m 1 /+ 250 250 0
 *   ./bot-opos6ul t /n 9 750 250  750 750  250 750  250 250 /m 2 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   ./bot-opos6ul t /n 9 300 100  300 300  100 300  100 100 /m 0 /s 20 /+ 100 100 0
 *   ./bot-opos6ul t /n 9 300 100  300 300  100 300  100 100 /m 1 /s 20 /+ 100 100 0
 *   ./bot-opos6ul t /n 9 300 100  300 300  100 300  100 100 /m 2 /s 20 /+ 100 100 0
 */

#include "O_AsservWaypointTest.hpp"

#include <cstdlib>
#include <string>
#include <vector>

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

void O_AsservWaypointTest::configureConsoleArgs(int argc, char **argv)
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	// Jusqu'a 5 waypoints (x, y)
	robot.getArgs().addArgument("x1", "waypoint 1 x mm");
	robot.getArgs().addArgument("y1", "waypoint 1 y mm");

	robot.getArgs().addArgument("x2", "waypoint 2 x mm", "-1");
	robot.getArgs().addArgument("y2", "waypoint 2 y mm", "-1");

	robot.getArgs().addArgument("x3", "waypoint 3 x mm", "-1");
	robot.getArgs().addArgument("y3", "waypoint 3 y mm", "-1");

	robot.getArgs().addArgument("x4", "waypoint 4 x mm", "-1");
	robot.getArgs().addArgument("y4", "waypoint 4 y mm", "-1");

	robot.getArgs().addArgument("x5", "waypoint 5 x mm", "-1");
	robot.getArgs().addArgument("y5", "waypoint 5 y mm", "-1");

	// PathMode
	Arguments::Option cOptMode('m', "PathMode");
	cOptMode.addArgument("mode", "0:STOP 1:CHAIN 2:CHAIN_NONSTOP", "0");
	robot.getArgs().addOption(cOptMode);

	// Speed
	Arguments::Option cOptSpeed('s', "speed en %");
	cOptSpeed.addArgument("speed", "speed en %", "40");
	robot.getArgs().addOption(cOptSpeed);

	// Detection
	Arguments::Option cOptDetect('B', "Detection Balise");
	cOptDetect.addArgument("detection", "[0-1]", "0");
	robot.getArgs().addOption(cOptDetect);

	// Position initiale
	Arguments::Option cOpt('+', "Coordinates x,y,a");
	cOpt.addArgument("coordx", "coord x mm", "300.0");
	cOpt.addArgument("coordy", "coord y mm", "300.0");
	cOpt.addArgument("coorda", "coord teta deg", "0.0");
	robot.getArgs().addOption(cOpt);

	Arguments::Option cOptMultiplier('M', "simu speed multiplier (0=instantane, 1.0=temps reel)");
	cOptMultiplier.addArgument("multiplier", "multiplier", "1.0");
	robot.getArgs().addOption(cOptMultiplier);

	robot.parseConsoleArgs(argc, argv);
}

void O_AsservWaypointTest::run(int argc, char **argv)
{
	logger().info() << "N " << this->position() << " - Executing - " << this->desc() << logs::end;
	configureConsoleArgs(argc, argv);

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	Arguments args = robot.getArgs();

	// Parse waypoints
	std::vector<Waypoint> waypoints;
	const char* xNames[] = {"x1", "x2", "x3", "x4", "x5"};
	const char* yNames[] = {"y1", "y2", "y3", "y4", "y5"};

	for (int i = 0; i < 5; i++)
	{
		float wx = atof(args[xNames[i]].c_str());
		float wy = atof(args[yNames[i]].c_str());
		if (wx == -1 && wy == -1)
			break;
		waypoints.push_back({wx, wy, false});
	}

	if (waypoints.empty())
	{
		logger().error() << "Aucun waypoint defini !" << logs::end;
		return;
	}

	// Parse options
	int modeInt = atoi(args['m']["mode"].c_str());
	PathMode mode = STOP;
	if (modeInt == 1) mode = CHAIN;
	else if (modeInt == 2) mode = CHAIN_NONSTOP;

	int s = atoi(args['s']["speed"].c_str());
	int B = atoi(args['B']["detection"].c_str());

	float coordx = atof(args['+']["coordx"].c_str());
	float coordy = atof(args['+']["coordy"].c_str());
	float coorda_deg = atof(args['+']["coorda"].c_str());
	float multiplier = atof(args['M']["multiplier"].c_str());

	// Log config
	const char* modeNames[] = {"STOP", "CHAIN", "CHAIN_NONSTOP"};
	logger().info() << "PathMode=" << modeNames[modeInt]
	                << " speed=" << s << " detection=" << B
	                << " simuSpeed=" << multiplier << logs::end;
	logger().info() << "Position initiale cx=" << coordx << " cy=" << coordy
	                << " ca=" << coorda_deg << logs::end;
	for (size_t i = 0; i < waypoints.size(); i++)
	{
		logger().info() << "  wp[" << i << "] x=" << waypoints[i].x
		                << " y=" << waypoints[i].y << logs::end;
	}

	// Init asserv : setPositionAndColor AVANT startMotionTimerAndOdo
	robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, (bool)(robot.getMyColor() != PMXYELLOW));
	robot.asserv().startMotionTimerAndOdo(false);
	robot.asserv().assistedHandling();
	robot.asserv().setSimuSpeedMultiplier(multiplier);

	robot.svgPrintPosition();

	// Init actions + detection
	robot.actions().start();
	if (B == 1)
	{
		robot.actions().sensors().startSensorsThread(20);
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	}
	else
	{
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	}
	robot.chrono().start();

	// Vitesse
	robot.asserv().setMaxSpeed(true, s, s);

	// Navigator
	Navigator nav(&robot, &robot.ia().iAbyPath());

	RetryPolicy policy = RetryPolicy::aggressive();
	policy.rotateIgnoringOpponent = (B == 0);

	// Execute
	logger().info() << "=== START manualPath ===" << logs::end;

	TRAJ_STATE ts = nav.manualPath(waypoints, policy, mode);

	logger().info() << "=== RESULT: " << robot.asserv().getTraj(ts) << " ===" << logs::end;

	// Fin
	robot.svgPrintPosition();
	robot.asserv().freeMotion();

	ROBOTPOSITION pos = robot.asserv().pos_getPosition();
	logger().info() << "time=" << robot.chrono().getElapsedTimeInMilliSec() << "ms"
	                << " x=" << pos.x << " y=" << pos.y
	                << " deg=" << pos.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();
	robot.svgPrintEndOfFile();
	logger().info() << robot.getID() << " " << this->name() << " Happy End N " << this->position() << logs::end;
}
