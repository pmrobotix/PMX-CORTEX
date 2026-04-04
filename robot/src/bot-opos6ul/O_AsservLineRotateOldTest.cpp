/*!
 * \file
 * \brief Implémentation de la classe O_AsservLineRotateOldTest.
 *
 * Exemples de lignes de commande (args: d a back [d2..d5 a2..a5 back2..back5]) :
 * Options : /m 0=relatif(defaut) 1=absolu  /s vitesse%  /r repetitions  /p pathfinding  /B detection  /+ x y a
 *
 *   --- Mode relatif (defaut, /m 0) - marche avant ---
 *
 *   # Ligne droite 300mm
 *   ./bot-opos6ul t /n 8 300
 *
 *   # Carré de 300mm (rotation relative +90°)
 *   ./bot-opos6ul t /n 8 300 90 0  300 90 0  300 90 0  300 90 0 /+ 500 500 0
 *
 *   # Carré de 300mm répété 3 fois
 *   ./bot-opos6ul t /n 8 300 90 0  300 90 0  300 90 0  300 90 0 /r 3 /+ 500 500 0
 *
 *   # Aller-retour 500mm (demi-tour relatif 180°), 5 fois
 *   ./bot-opos6ul t /n 8 500 180 0 /r 5 /+ 500 500 0
 *
 *   # Triangle équilatéral 400mm (rotation relative +120°)
 *   ./bot-opos6ul t /n 8 400 120 0  400 120 0  400 120 0 /+ 500 500 0
 *
 *   --- Mode relatif - marche arrière (back=1) ---
 *
 *   # Carré de 300mm en marche arrière : demi-tour puis 4 segments back
 *   ./bot-opos6ul t /n 8 0 180 0  300 90 1  300 90 1  300 90 1  300 90 1 /+ 500 500 0
 *
 *   # Carré de 300mm en marche arrière répété 3 fois
 *   ./bot-opos6ul t /n 8 0 180 0  300 90 1  300 90 1  300 90 1  300 90 1 /r 3 /+ 500 500 0
 *
 *   # Ligne 200mm en marche arrière, vitesse 30%
 *   ./bot-opos6ul t /n 8 200 -1 1 /s 30 /+ 500 500 0
 *
 *   --- Mode absolu (/m 1) ---
 *
 *   # Carré de 300mm (orientations absolues 90°, 180°, 270°, 0°)
 *   ./bot-opos6ul t /n 8 300 90 0  300 180 0  300 270 0  300 0 0 /m 1 /+ 500 500 0
 *
 *   # Ligne 400mm puis orientation absolue vers 45°
 *   ./bot-opos6ul t /n 8 400 45 0 /m 1 /+ 500 500 0
 */

#include "O_AsservLineRotateOldTest.hpp"

#include <cstdlib>
#include <string>

#include "action/Sensors.hpp"
#include "utils/Arguments.hpp"
#include "interface/AAsservDriver.hpp"
#include "ia/IAbyPath.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_AsservLineRotateOldTest::configureConsoleArgs(int argc, char **argv)
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
	Arguments::Option cOptPMode('p', "mode pathfinding");
	cOptPMode.addArgument("pmode", "mode number", "0");
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

void O_AsservLineRotateOldTest::run(int argc, char **argv)
{
	logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
	configureConsoleArgs(argc, argv);

	float dd = 0.0;
	float aa = 0.0;
	bool bback = false;

	float d = 0.0;
	float a = 0.0;
	bool back = false;

	float d2 = 0.0;
	float a2 = 0.0;
	bool back2 = false;

	float d3 = 0.0;
	float a3 = 0.0;
	bool back3 = false;

	float d4 = 0.0;
	float a4 = 0.0;
	bool back4 = false;

	float d5 = 0.0;
	float a5 = 0.0;
	bool back5 = false;

	int B = 0;
	int m = 0;
	int s = 0;
	int pathfindingMode = 0;

	float coordx = 0.0;
	float coordy = 0.0;
	float coorda_deg = 0.0;

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	Arguments args = robot.getArgs();

	if (args["d"] != "0")
	{
		d = atof(args["d"].c_str());
		logger().info() << "Arg d set " << args["d"] << ", d = " << d << logs::end;
	}
	if (args["a"] != "0")
	{
		a = atof(args["a"].c_str());
		logger().info() << "Arg a set " << args["a"] << ", a = " << a << logs::end;
	}
	if (args["back"] != "0")
	{
		back = atoi(args["back"].c_str());
		logger().info() << "Arg back set " << args["back"] << ", back = " << back << logs::end;
	}
	if (args["d2"] != "0")
	{
		d2 = atof(args["d2"].c_str());
		logger().info() << "Arg d2 set " << args["d2"] << ", d2 = " << d2 << logs::end;
	}
	if (args["a2"] != "0")
	{
		a2 = atof(args["a2"].c_str());
		logger().info() << "Arg a2 set " << args["a2"] << ", a2 = " << a2 << logs::end;
	}
	if (args["back2"] != "0")
	{
		back2 = atoi(args["back2"].c_str());
		logger().info() << "Arg back2 set " << args["back2"] << ", back2 = " << back2 << logs::end;
	}

	if (args["d3"] != "0")
	{
		d3 = atof(args["d3"].c_str());
		logger().info() << "Arg d3 set " << args["d3"] << ", d3 = " << d3 << logs::end;
	}
	if (args["a3"] != "0")
	{
		a3 = atof(args["a3"].c_str());
		logger().info() << "Arg a3 set " << args["a3"] << ", a3 = " << a3 << logs::end;
	}
	if (args["back3"] != "0")
	{
		back3 = atoi(args["back3"].c_str());
		logger().info() << "Arg back3 set " << args["back3"] << ", back3 = " << back3 << logs::end;
	}

	if (args["d4"] != "0")
	{
		d4 = atof(args["d4"].c_str());
		logger().info() << "Arg d4 set " << args["d4"] << ", d4 = " << d4 << logs::end;
	}
	if (args["a4"] != "0")
	{
		a4 = atof(args["a4"].c_str());
		logger().info() << "Arg a4 set " << args["a4"] << ", a4 = " << a4 << logs::end;
	}
	if (args["back4"] != "0")
	{
		back4 = atoi(args["back4"].c_str());
		logger().info() << "Arg back4 set " << args["back4"] << ", back4 = " << back4 << logs::end;
	}

	if (args["d5"] != "0")
	{
		d5 = atof(args["d5"].c_str());
		logger().info() << "Arg d5 set " << args["d5"] << ", d5 = " << d5 << logs::end;
	}
	if (args["a5"] != "0")
	{
		a5 = atof(args["a5"].c_str());
		logger().info() << "Arg a5 set " << args["a5"] << ", a5 = " << a5 << logs::end;
	}
	if (args["back5"] != "0")
	{
		back5 = atoi(args["back5"].c_str());
		logger().info() << "Arg back5 set " << args["back5"] << ", back5 = " << back5 << logs::end;
	}

	B = atoi(args['B']["detection"].c_str());
	logger().info() << "Arg B set " << args['B']["detection"] << ", B = " << B << logs::end;

	s = atoi(args['s']["speed"].c_str());
	logger().info() << "Arg s set " << args['s']["speed"] << ", s = " << s << logs::end;

	m = atoi(args['m']["mode"].c_str());
	logger().info() << "Arg m set " << args['m']["mode"] << ", m = " << m << logs::end;

	pathfindingMode = atoi(args['p']["pmode"].c_str());
	logger().info() << "Arg p set " << args['p']["pmode"] << ", pathfindingMode = " << pathfindingMode << logs::end;

	int repeat = atoi(args['r']["repeat"].c_str());
	logger().info() << "Arg r set " << args['r']["repeat"] << ", repeat = " << repeat << logs::end;

	coordx = atof(args['+']["coordx"].c_str());
	coordy = atof(args['+']["coordy"].c_str());
	coorda_deg = atof(args['+']["coorda"].c_str());
	logger().info() << "COORD avec cx=" << coordx << " cy=" << coordy << " coorda=" << coorda_deg << logs::end;

	robot.asserv().startMotionTimerAndOdo(false);
	robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, (bool) (robot.getMyColor() != PMXYELLOW));
	robot.asserv().assistedHandling();

	ROBOTPOSITION pos = robot.asserv().pos_getPosition();
	logger().info() << "time= " << robot.chrono().getElapsedTimeInMilliSec() << "ms ; " << " px=" << pos.x << " py="
			<< pos.y << " pa_deg=" << pos.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();

	robot.actions().start();
	if (B == 1)
	{
		//detection adverse
		robot.actions().sensors().addTimerSensors(62);

		robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	} else
	{
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	}
	robot.chrono().start();

	//vitesse reduite
	robot.asserv().setMaxSpeed(true, s, s);

	TRAJ_STATE ts = TRAJ_IDLE;

	for (int rep = 1; rep <= repeat; rep++)
	{
	logger().info() << "=== REPETITION " << rep << "/" << repeat << " ===" << logs::end;

	for (int nb = 1; nb <= 5; nb++)
	{

		if (nb == 1)
		{
			bback = back;
			aa = a;
			dd = d;
		}
		if (nb == 2)
		{
			bback = back2;
			aa = a2;
			dd = d2;
		}
		if (nb == 3)
		{
			bback = back3;
			aa = a3;
			dd = d3;
		}
		if (nb == 4)
		{
			bback = back4;
			aa = a4;
			dd = d4;
		}
		if (nb == 5)
		{
			bback = back5;
			aa = a5;
			dd = d5;
		}

		if (dd != -1)
		{
			logger().info() << "=> TRAJET n°: " << nb << logs::end;

			if (bback == 1)
			{
				dd = -dd;
			}
			if (pathfindingMode == 0)
			{
				logger().info() <<  " doline 1 " << logs::end;
				ts = robot.asserv().doLine(dd);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " =====  CONFIRMED AFTER n ;WHAT TO DO ?"
							<< logs::end;
					robot.asserv().resetEmergencyOnTraj(" doLine: " + ts);
				}
				/*
				utils::sleep_for_micros(1000000);
				logger().info() <<  " doline 2 " << logs::end;
				ts = robot.asserv().doLine(dd);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " =====  CONFIRMED AFTER n ;WHAT TO DO ?"
							<< logs::end;
					robot.asserv().resetEmergencyOnTraj(" doLine: " + ts);
				}
				utils::sleep_for_micros(1000000);
				logger().info() <<  " doline 3 " << logs::end;
				ts = robot.asserv().doLine(dd);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " =====  CONFIRMED AFTER n ;WHAT TO DO ?"
							<< logs::end;
					robot.asserv().resetEmergencyOnTraj(" doLine: " + ts);
				}
				utils::sleep_for_micros(1000000);
				logger().info() <<  " doline 4 " << logs::end;
				ts = robot.asserv().doLine(dd);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " =====  CONFIRMED AFTER n ;WHAT TO DO ?"
							<< logs::end;
					robot.asserv().resetEmergencyOnTraj(" doLine: " + ts);
				}*/

			} else if (pathfindingMode == 1)
			{
				Navigator nav(&robot);
				RetryPolicy policyLine = { 2000000, 5, 5, 50, 0, false, false };
				ts = nav.line(dd, policyLine);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts) << " =====  CONFIRMED AFTER n ;WHAT TO DO ?"
							<< logs::end;
					robot.asserv().resetEmergencyOnTraj(" whileDoLine: " + ts);

					//on recule de 2cm
					if (bback == 1)
					{
						ts = robot.asserv().doLine(20);
					} else
						ts = robot.asserv().doLine(-20);

					//on decide de continuer ici au niveau de la prise de decision apres les n essais.
					robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
					robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
				}

			} else if (pathfindingMode == 2)
			{
				//while avec pathfinding
			}
			robot.svgPrintPosition();
		}
		if (m == 0) //mouvement en relatif
		{
			if (aa != -1)
			{
				ts = robot.asserv().doRelativeRotateDeg(aa);
				if (ts >= TRAJ_INTERRUPTED)
				{
					logger().info() << robot.asserv().getTraj(ts)
							<< " doRelativeRotateDeg =====  CONFIRMED AFTER 1 turn ;WHAT TO DO ?" << logs::end;
					robot.asserv().resetEmergencyOnTraj(" doRelativeRotateDeg: " + ts);
				}
				robot.svgPrintPosition();
			}
		} else if (m == 1) //mode angle absolu
		{
			ts = robot.asserv().doAbsoluteRotateTo(aa);
			if (ts >= TRAJ_INTERRUPTED)
			{
				logger().info() << robot.asserv().getTraj(ts)
						<< " doAbsoluteRotateTo =====  CONFIRMED AFTER 1 turn ;WHAT TO DO ?" << logs::end;
				robot.asserv().resetEmergencyOnTraj(" doAbsoluteRotateTo: " + ts);
			}
			robot.svgPrintPosition();
		}
	}
	} // fin repetition

	robot.svgPrintPosition();

	robot.asserv().freeMotion();

	pos = robot.asserv().pos_getPosition();
	logger().info() << "time= " << robot.chrono().getElapsedTimeInMilliSec() << "ms ; " << " x=" << pos.x << " y="
			<< pos.y << " deg=" << pos.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();
	robot.svgPrintEndOfFile();
	logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
