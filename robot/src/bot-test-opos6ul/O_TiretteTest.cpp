/*!
 * \file
 * \brief Implémentation de la classe O_TiretteTest.
 */

#include "O_TiretteTest.hpp"

#include <cstdlib>

#include "utils/Arguments.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

using namespace std;

void O_TiretteTest::configureConsoleArgs(int argc, char** argv)
{
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.getArgs().addArgument("timeout", "duree du test en secondes", "10");
    robot.parseConsoleArgs(argc, argv);
}

void O_TiretteTest::run(int argc, char** argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;
    configureConsoleArgs(argc, argv);

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    Arguments args = robot.getArgs();
    int timeout = atoi(args["timeout"].c_str());

    logger().info() << "O_TiretteTest timeout=" << timeout << "s" << logs::end;

    utils::Chronometer chrono("O_TiretteTest");
    chrono.start();

    while (chrono.getElapsedTimeInSec() < timeout)
    {
        logger().info() << "pressed : " << robot.actions().tirette().pressed() << logs::end;
        utils::sleep_for_micros(200000); // 200ms entre chaque lecture
    }

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
