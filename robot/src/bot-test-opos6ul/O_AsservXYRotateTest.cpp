/*!
 * \file
 * \brief Implémentation de la classe O_AsservXYRotateTest.
 */

#include "O_AsservXYRotateTest.hpp"

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

void O_AsservXYRotateTest::configureConsoleArgs(int argc, char **argv)
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	robot.getArgs().addArgument("s", "speed en %");
	robot.getArgs().addArgument("d", "distance mm");
	robot.getArgs().addArgument("x", "x destination mm", "0");
	robot.getArgs().addArgument("y", "y destination mm", "0");
	robot.getArgs().addArgument("a", "angle degre", "0");
	robot.getArgs().addArgument("back", "back:0,1", "0");
	robot.getArgs().addArgument("nb", "nb of time", "1");

	Arguments::Option cOpt('+', "Coordinates x,y,a");
	cOpt.addArgument("coordx", "coord x mm", "300.0");
	cOpt.addArgument("coordy", "coord y mm", "300.0");
	cOpt.addArgument("coorda", "coord teta deg", "0.0");
	robot.getArgs().addOption(cOpt);

	//reparse arguments
	robot.parseConsoleArgs(argc, argv);
}

void O_AsservXYRotateTest::run(int argc, char **argv)
{
	logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
	configureConsoleArgs(argc, argv);

	int left;
	int right;
	int s = 0;
	float d = 0.0;
	float x = 0.0;
	float y = 0.0;
	int nb = 0;
	bool back = false;
	float a = 0.0;
	float coordx = 0.0;
	float coordy = 0.0;
	float coorda_deg = 0.0;

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	Arguments args = robot.getArgs();
	if (args["s"] != "0")
	{
		s = atoi(args["s"].c_str());
		logger().debug() << "Arg s set " << args["s"] << ", s = " << s << logs::end;
	}

	if (args["d"] != "0")
	{
		d = atof(args["d"].c_str());
		logger().info() << "Arg d set " << args["d"] << ", d = " << d << logs::end;
	}

	if (args["x"] != "0")
	{
		x = atof(args["x"].c_str());
		logger().info() << "Arg x set " << args["x"] << ", x = " << x << logs::end;
	}

	if (args["y"] != "0")
	{
		y = atof(args["y"].c_str());
		logger().info() << "Arg y set " << args["y"] << ", y = " << y << logs::end;
	}

	if (args["a"] != "0")
	{
		a = atof(args["a"].c_str());
		logger().info() << "Arg a set " << args["a"] << ", a = " << a << logs::end;
	}

	if (args["nb"] != "0")
	{
		nb = atoi(args["nb"].c_str());
		logger().info() << "Arg nb set " << args["nb"] << ", nb = " << nb << logs::end;
	}
	coordx = atof(args['+']["coordx"].c_str());
	coordy = atof(args['+']["coordy"].c_str());
	coorda_deg = atof(args['+']["coorda"].c_str());

	if (args["back"] != "0")
	{
		back = atoi(args["back"].c_str());
		logger().info() << "Arg back set " << args["back"] << ", back = " << back << logs::end;
	}

	logger().info() << "COORD avec cx=" << coordx << " cy=" << coordy << " coorda=" << coorda_deg << logs::end;

	// setPositionAndColor AVANT startMotionTimerAndOdo
	robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, robot.isMatchColor());
	robot.asserv().startMotionTimerAndOdo(false);
	robot.asserv().assistedHandling();

	robot.asserv().getEncodersCounts(&right, &left);
	ROBOTPOSITION p = robot.asserv().pos_getPosition();
	logger().info() << "time= " << robot.chrono().getElapsedTimeInMilliSec() << "ms ; " << " px=" << p.x << " py="
			<< p.y << " pa_deg=" << p.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();

	//detection adverse
	robot.actions().start();
	robot.actions().sensors().startSensorsThread(20);
	robot.chrono().start();
	robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

	//vitesse reduite
	robot.asserv().setSpeed(s);

	Navigator nav(&robot, &robot.ia().iAbyPath());
	TRAJ_STATE ts = TRAJ_IDLE;

	for (int num = 1; num <= nb; num++)
	{
		logger().info() << "Go essai num= " << num << " / " << nb << logs::end;
		ts = TRAJ_IDLE;
		if (d != 0)
		{
			logger().info() << "go ...d=" << d << "mm" << logs::end;

			//calcul de coord
			float x_dest = robot.asserv().pos_getX_mm() + cos(robot.asserv().pos_getTheta()) * d;
			float y_dest = robot.asserv().pos_getY_mm() + sin(robot.asserv().pos_getTheta()) * d;
			if (back == 0)
			{
				robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
				robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

				bool frontcenter = robot.actions().sensors().getAvailableFrontCenter();
				logger().info() << "frontcenter=" << frontcenter << " " << logs::end;

				RetryPolicy policyFwd = { 1000000, 5, 10, 0, 0, false };
				ts = nav.moveForwardTo(x_dest, y_dest, policyFwd);
				if (ts == TRAJ_INTERRUPTED)
				{
					logger().info() << "===== TRAJ_NEAR_OBSTACLE CONFIRMED" << logs::end;

					logger().info() << "Attente 3 sec!" << logs::end;
					utils::sleep_for_secs(3);
					robot.asserv().resetEmergencyOnTraj("===== TRAJ_NEAR_OBSTACLE CONFIRMED");
				}
				if (ts == TRAJ_COLLISION)
				{
					logger().info() << "===== COLLISION ASSERV CONFIRMED" << logs::end;
					robot.asserv().stopMotors();

					logger().info() << "Attente 3 sec!" << logs::end;
					utils::sleep_for_secs(3);
					robot.asserv().resetEmergencyOnTraj("===== COLLISION ASSERV CONFIRMED");
				}
				if ((ts != TRAJ_INTERRUPTED) && (ts != TRAJ_COLLISION) && (ts != TRAJ_FINISHED))
				{
					logger().info() << "=====> cas pas normal ts=" << ts << logs::end;
					robot.asserv().resetEmergencyOnTraj("===== OTHER CASE CONFIRMED");
				}

			} else
			{
				robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
				robot.actions().sensors().setIgnoreBackNearObstacle(true, false, true);

				RetryPolicy policyBack = { 1000000, 3, 2, 0, 0, false };
				ts = nav.moveBackwardTo(robot.asserv().pos_getX_mm() + d,
						robot.asserv().pos_getY_mm(), policyBack);
				if (ts == TRAJ_INTERRUPTED)
				{
					logger().info() << "===== TRAJ_NEAR_OBSTACLE CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj("==== TRAJ_NEAR_OBSTACLE CONFIRMED");
				}
				if (ts == TRAJ_COLLISION)
				{
					logger().info() << "===== COLLISION ASSERV CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj("===== COLLISION ASSERV CONFIRMED");
				}
			}
		}
		ts = TRAJ_IDLE;
		if (a != 0.0)
		{
			logger().info() << "go Rotate..." << a << " deg" << logs::end;
			robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
			robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

			RetryPolicy policyRot = { 1000000, 2, 2, 0, 0, false };
			ts = nav.rotateAbsDeg(a, policyRot);

			if (ts == TRAJ_INTERRUPTED)
			{
				logger().error() << "===== TRAJ_NEAR_OBSTACLE FINAL" << logs::end;
				robot.asserv().resetEmergencyOnTraj("rotate ===== TRAJ_NEAR_OBSTACLE FINAL");
			}
			if (ts == TRAJ_COLLISION)
			{
				logger().error() << "===== COLLISION ASSERV FINAL" << logs::end;
				robot.asserv().resetEmergencyOnTraj("rotate ===== COLLISION ASSERV FINAL");
			}

			robot.svgPrintPosition();
		}
		ts = TRAJ_IDLE;
		if (!(x == 0.0 && y == 0.0))
		{
			if (!back)
			{
				logger().info() << "go Forward... x=" << x << ", y=" << y << logs::end;
				robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
				robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

				RetryPolicy policyFwd2 = { 1000000, 20, 2, 0, 0, false };
				ts = nav.moveForwardTo(x, y, policyFwd2);

				if (ts == TRAJ_INTERRUPTED)
				{
					logger().error() << "===== TRAJ_NEAR_OBSTACLE CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj(
							"nav.moveForwardTo FINAL TRAJ_NEAR_OBSTACLE");
				}
				if (ts == TRAJ_COLLISION)
				{
					logger().error() << "===== COLLISION ASSERV CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj(
							"nav.moveForwardTo FINAL TRAJ_COLLISION");
				}

				robot.svgPrintPosition();
			} else
			{
				logger().info() << "go Backward... x=" << x << ", y=" << y << logs::end;
				robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
				robot.actions().sensors().setIgnoreBackNearObstacle(true, false, true);

				RetryPolicy policyBack2 = { 1000000, 20, 3, 0, 0, false };
				ts = nav.moveBackwardTo(x, y, policyBack2);

				if (ts == TRAJ_INTERRUPTED)
				{
					logger().error() << "===== TRAJ_NEAR_OBSTACLE CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj(
							"nav.moveBackwardTo FINAL TRAJ_NEAR_OBSTACLE");
				}
				if (ts == TRAJ_COLLISION)
				{
					logger().error() << "===== COLLISION ASSERV CONFIRMED" << logs::end;
					robot.asserv().resetEmergencyOnTraj(
							"nav.moveBackwardTo FINAL TRAJ_COLLISION");
				}

				robot.svgPrintPosition();
			}
		}
		ts = TRAJ_IDLE;
		if (a != 0)
		{
			robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
			robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

			RetryPolicy policyRot2 = { 1000000, 2, 2, 0, 0, false };
			ts = nav.rotateAbsDeg(a, policyRot2);

			if (ts == TRAJ_INTERRUPTED)
			{
				logger().error() << "===== TRAJ_NEAR_OBSTACLE CONFIRMED" << logs::end;
				robot.asserv().resetEmergencyOnTraj();
			}
			if (ts == TRAJ_COLLISION)
			{
				logger().error() << "===== COLLISION ASSERV CONFIRMED" << logs::end;
				robot.asserv().resetEmergencyOnTraj();
			}

			robot.svgPrintPosition();
		}
	}

	robot.svgPrintPosition();

	robot.asserv().freeMotion();
	robot.asserv().setSpeed(100);   // restore vitesse nominale en fin de test
	p = robot.asserv().pos_getPosition();
	logger().info() << "time= " << robot.chrono().getElapsedTimeInMilliSec() << "ms ; " << " x=" << p.x << " y=" << p.y
			<< " deg=" << p.theta * 180.0 / M_PI << logs::end;

	robot.svgPrintPosition();
	robot.svgPrintEndOfFile();
	logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
