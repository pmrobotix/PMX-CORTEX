/*!
 * \file
 * \brief Implémentation de la classe O_GroveColorTest.
 */

#include "O_GroveColorTest.hpp"

#include <string>

#include "log/Logger.hpp"
#include "OPOS6UL_RobotExtended.hpp"

void O_GroveColorTest::run(int argc, char** argv)
{
    logger().info() << "N° " << this->position() << " - Executing - " << this->desc() << logs::end;

    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    //TODO implémenter le test capteur couleur

    logger().info() << robot.getID() << " " << this->name() << " Happy End" << " N° " << this->position() << logs::end;
}
