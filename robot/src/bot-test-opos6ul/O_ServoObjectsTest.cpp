/*!
 * \file
 * \brief Implémentation de la classe O_ServoObjectsTest.
 */

#include "O_ServoObjectsTest.hpp"

#include <unistd.h>
#include <string>

#include "utils/Arguments.hpp"
#include "Robot.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

using namespace std;

void O_ServoObjectsTest::configureConsoleArgs(int argc, char **argv) //surcharge
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	robot.getArgs().addArgument("action", "action a faire");
	robot.getArgs().addArgument("vitesse", "vitesse lancer 0,127", "0");

	//reparse arguments
	robot.parseConsoleArgs(argc, argv);
}

void O_ServoObjectsTest::run(int argc, char **argv)
{
	logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;

	configureConsoleArgs(argc, argv); //on appelle les parametres specifiques pour ce test

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	Arguments args = robot.getArgs();

	string action = "";
	if (args["action"] != "0")
	{
		action = args["action"].c_str();
		logger().info() << "Arg action set " << args["action"] << ", action = " << action << logs::end;
	}

	int vitesse = 0;
	if (args["vitesse"] != "0")
	{
		vitesse = atoi(args["vitesse"].c_str());
		logger().info() << "Arg action set " << args["vitesse"] << ", vitesse = " << vitesse << logs::end;
	}

	logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;

	robot.actions().start();

	if (action == "GETALL")
	{
		//TODO a faire en generique avec boucle for
		robot.actions().servos().release(robot.actions().AX12_SERVO_BRAS_D);
		robot.actions().servos().release(robot.actions().AX12_SERVO_BRAS_G);

		while (1)
		{
			logger().info() << "AX12_SERVO_BRAS_D       N° " << robot.actions().AX12_SERVO_BRAS_D << " pos= "
					<< robot.actions().servos().getPulseWidth(robot.actions().AX12_SERVO_BRAS_D) << logs::end;
			logger().info() << "AX12_SERVO_BRAS_G       N° " << robot.actions().AX12_SERVO_BRAS_G << " pos= "
					<< robot.actions().servos().getPulseWidth(robot.actions().AX12_SERVO_BRAS_G) << logs::end;
			logger().info() << logs::end;
			logger().info() << logs::end;
			logger().info() << logs::end;
			utils::sleep_for_millis(500);
		}
	}

	if (action == "TEST")
	{
		while (1)
		{
			robot.actions().ax12_bras_droit(-1);
			robot.actions().ax12_bras_droit(-1);
			robot.actions().ax12_bras_droit_init(-1);
			robot.actions().ax12_bras_droit_init(-1);
			robot.actions().ax12_bras_gauche_init(0);
			//utils::sleep_for_secs(2);

			robot.actions().ax12_bras_gauche(-1);
			robot.actions().ax12_bras_gauche(-1);
			robot.actions().ax12_bras_gauche_init(-1);
			robot.actions().ax12_bras_gauche_init(-1);
			robot.actions().ax12_bras_droit_init(0);
		}
	}

	

	if (action == "GO")
	{
		ButtonTouch b = BUTTON_NONE;
		bool fL = true;
		bool fR = true;
		while (b != BUTTON_BACK_KEY)
		{
			if (b == BUTTON_LEFT_KEY)
			{
				//robot.actions().ax12_up_L(2000, 0);

				if (fL)
				{
					robot.actions().ax12_bras_droit(0);
					//robot.actions().ax12_close_L(1000, -1);

				} else
				{
					robot.actions().ax12_bras_droit_init(0);
					//robot.actions().ax12_open_L(1000, -1);

				}
				fL = !fL;
			}

			if (b == BUTTON_RIGHT_KEY)
			{
				//robot.actions().ax12_up_R(2000, 0);

				if (fR)
				{
					robot.actions().ax12_bras_droit(0);
					robot.actions().ax12_bras_gauche(0);
					//robot.actions().ax12_close_R(1000, -1);
				} else
				{
					robot.actions().ax12_bras_gauche_init(0);
					robot.actions().ax12_bras_droit_init(0);
					//robot.actions().ax12_open_R(1000, -1);
				}
				fR = !fR;
			}

			if (b == BUTTON_UP_KEY)
			{
				//robot.actions().ax12_up(2000, 0);
			}
			if (b == BUTTON_DOWN_KEY)
			{
				robot.actions().ax12_init();
			}

			b = robot.actions().buttonBar().waitOneOfAllPressed();

			utils::sleep_for_micros(10000);
		}
	}
	
	logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;

}

