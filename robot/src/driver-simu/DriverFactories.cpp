/*!
 * \file
 * \brief Factory create() pour le build SIMU.
 *
 * En mode SIMU, les create() instancient toujours les stubs.
 * Ce fichier regroupe les factories pour eviter les symbol duplicates
 * avec les fichiers driver individuels.
 */

#include "interface/ALedDriver.hpp"
#include "LedDriver.hpp"

ALedDriver* ALedDriver::create(std::string, int nb)
{
    return new LedDriverSimu(nb);
}

#include "interface/ALcdShieldDriver.hpp"
#include "LcdShieldDriver.hpp"

ALcdShieldDriver* ALcdShieldDriver::create(std::string botId)
{
    return new LcdShieldDriverSimu();
}

#include "interface/AButtonDriver.hpp"
#include "ButtonDriver.hpp"

AButtonDriver* AButtonDriver::create()
{
    return new ButtonDriverSimu();
}

#include "interface/ASwitchDriver.hpp"
#include "SwitchDriver.hpp"

ASwitchDriver* ASwitchDriver::create(std::string)
{
    static SwitchDriverSimu *instance = new SwitchDriverSimu();
    return instance;
}

#include "interface/ASensorsDriver.hpp"
#include "SensorsDriver.hpp"

ASensorsDriver* ASensorsDriver::create(std::string, ARobotPositionShared *aRobotPositionShared)
{
    return new SensorsDriverSimu(aRobotPositionShared);
}

#include "interface/AServoDriver.hpp"
#include "ServoDriver.hpp"

AServoDriver* AServoDriver::create()
{
    static ServoDriverSimu *instance = new ServoDriverSimu();
    return instance;
}

#include "interface/AAsservDriver.hpp"
#include "AsservDriver.hpp"

AAsservDriver* AAsservDriver::create(std::string botid, ARobotPositionShared *aRobotPositionShared)
{
    static AsservDriverSimu *instance = new AsservDriverSimu(botid, aRobotPositionShared);
    return instance;
}

#include "interface/AColorDriver.hpp"
#include "ColorDriver.hpp"

AColorDriver* AColorDriver::create(std::string)
{
    return new ColorDriverSimu();
}
