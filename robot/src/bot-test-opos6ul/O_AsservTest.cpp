/*!
 * \file
 * \brief Implémentation de la classe O_AsservTest.
 */

#include "O_AsservTest.hpp"

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

void O_AsservTest::configureConsoleArgs(int argc, char **argv)
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	robot.getArgs().addArgument("s", "speed en %");
	robot.getArgs().addArgument("x", "x mm");
	robot.getArgs().addArgument("y", "y mm");
	robot.getArgs().addArgument("x2", "x2 mm", "0");
	robot.getArgs().addArgument("y2", "y2 mm", "0");
	robot.getArgs().addArgument("x3", "x3 mm", "0");
	robot.getArgs().addArgument("y3", "y3 mm", "0");

	Arguments::Option cOpt('+', "Coordinates x,y,a");
	cOpt.addArgument("coordx", "coord x mm", "200.0");
	cOpt.addArgument("coordy", "coord y mm", "200.0");
	cOpt.addArgument("coorda", "coord teta mm", "0.0");
	robot.getArgs().addOption(cOpt);

	//reparse again arguments for the specific test
	robot.parseConsoleArgs(argc, argv);
}

void O_AsservTest::run(int argc, char **argv)
{
	logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
	configureConsoleArgs(argc, argv);
	int s = 0;
	float x = 0.0;
	float y = 0.0;
	float x2 = 0.0;
	float y2 = 0.0;
	float x3 = 0.0;
	float y3 = 0.0;
	float coordx = 0.0;
	float coordy = 0.0;
	float coorda_deg = 0.0;
	TRAJ_STATE ts = TRAJ_IDLE;

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	Arguments args = robot.getArgs();

	if (args["s"] != "0")
	{
		s = atoi(args["s"].c_str());
		logger().debug() << "Arg s set " << args["s"] << ", s = " << s << logs::end;
	}
	if (args["x"] != "0")
	{
		x = atof(args["x"].c_str());
		logger().debug() << "Arg x set " << args["x"] << ", x = " << x << logs::end;
	}
	if (args["y"] != "0")
	{
		y = atof(args["y"].c_str());
		logger().debug() << "Arg y set " << args["y"] << ", y = " << y << logs::end;
	}

	if (args["x2"] != "0")
	{
		x2 = atof(args["x2"].c_str());
		logger().debug() << "Arg x2 set " << args["x2"] << ", x2 = " << x2 << logs::end;
	}
	if (args["y2"] != "0")
	{
		y2 = atof(args["y2"].c_str());
		logger().debug() << "Arg y2 set " << args["y2"] << ", y2 = " << y2 << logs::end;
	}

	if (args["x3"] != "0")
	{
		x3 = atof(args["x3"].c_str());
		logger().debug() << "Arg x3 set " << args["x3"] << ", x3 = " << x3 << logs::end;
	}
	if (args["y3"] != "0")
	{
		y3 = atof(args["y3"].c_str());
		logger().debug() << "Arg y3 set " << args["y3"] << ", y3 = " << y3 << logs::end;
	}

	coordx = atof(args['+']["coordx"].c_str());
	coordy = atof(args['+']["coordy"].c_str());
	coorda_deg = atof(args['+']["coorda"].c_str());

	logger().info() << "COORD avec x=" << coordx << " y=" << coordy << " a=" << coorda_deg << logs::end;

	// setPositionAndColor AVANT startMotionTimerAndOdo
	// (la couleur doit etre definie avant le set position car il applique le miroir,
	//  et le set position reset la Nucleo pour le debut de match)
	// Couleur respectee depuis CLI (defaut BLEU, /y pour JAUNE).
	robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, robot.isMatchColor());
	robot.asserv().startMotionTimerAndOdo(true);

	logger().info() << "setposition done:" << " x=" << robot.asserv().pos_getX_mm() << " y="
			<< robot.asserv().pos_getY_mm() << " a=" << robot.asserv().pos_getThetaInDegree() << " color="
			<< robot.getMyColor() << logs::end;
	robot.svgPrintPosition();

	robot.asserv().setMaxSpeed(true, s);

	//detection adverse
	robot.actions().start();
	robot.actions().sensors().startSensorsThread(20);
	robot.chrono().start();

	robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

	bool frontcenter = robot.actions().sensors().getAvailableFrontCenter();
	logger().info() << "frontcenter=" << frontcenter << " " << logs::end;

	Navigator nav(&robot, &robot.ia().iAbyPath());
	RetryPolicy policy = { 1000000, 5, 5, 0, 0, false };

	logger().info() << "GOTO x=" << x << " y=" << y << logs::end;
	ts = nav.moveForwardTo(x, y, policy);
	if (ts != TRAJ_FINISHED)
	{
		robot.logger().error() << " moveForwardTo  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
				<< logs::end;
		robot.asserv().resetEmergencyOnTraj();
	}
	logger().info() << "END GOTO1 ts=" << ts << logs::end;

	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);

	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_droit_init(-1);

	if (x2 != 0 && y2 != 0)
	{
		logger().info() << "GOTO2 x2=" << x2 << " y2=" << y2 << logs::end;

		RetryPolicy policy2 = { 3000000, 3, 3, 0, 0, false };
		TRAJ_STATE ts = nav.moveForwardTo(x2, y2, policy2);
		if (ts != TRAJ_FINISHED)
		{
			robot.logger().error() << " moveForwardTo x2,y2  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
					<< logs::end;
			robot.asserv().resetEmergencyOnTraj();
		}
		logger().info() << "END GOTO2 ts=" << ts << logs::end;
	}

	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);

	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_droit_init(-1);

	if (x3 != 0 && y3 != 0)
	{
		logger().info() << "GOTO2 x3=" << x3 << " y3=" << y3 << logs::end;

		RetryPolicy policy3 = { 3000000, 3, 3, 0, 0, false };
		TRAJ_STATE ts = nav.moveForwardTo(x3, y3, policy3);
		if (ts != TRAJ_FINISHED)
		{
			robot.logger().error() << " moveForwardTo x3,y3  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
					<< logs::end;
			robot.asserv().resetEmergencyOnTraj();
		}
		logger().info() << "END GOTO3 ts=" << ts << logs::end;
	}

	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_droit_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);

	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_gauche_init(-1);
	robot.actions().ax12_bras_droit_init(-1);

	logger().info() << "time= " << robot.chrono().getElapsedTimeInMilliSec() << "ms " << " x="
			<< robot.asserv().pos_getX_mm() << " y=" << robot.asserv().pos_getY_mm() << " a="
			<< robot.asserv().pos_getThetaInDegree() << logs::end;

	robot.svgPrintPosition();

	logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
