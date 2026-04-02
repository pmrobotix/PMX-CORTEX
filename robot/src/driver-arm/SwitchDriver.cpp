/*!
 * \file
 * \brief Implementation ARM du driver de switchs pour l'OPOS6UL.
 */

#include "SwitchDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/SwitchDriver.hpp"

#include <unistd.h>

#include "log/Logger.hpp"
#include "GpioPCA9555.hpp"

ASwitchDriver * ASwitchDriver::create(std::string)
{
	if (!HardwareConfig::instance().isEnabled("SwitchDriver")) {
		static SwitchDriverSimu *instance = new SwitchDriverSimu();
		return instance;
	}
	return new SwitchDriver();
}

SwitchDriver::SwitchDriver()
{
	bool c = GpioPCA9555::instance().begin();
	if (!c)
		logger().error() << "Hardware status: SwitchDriver is NOT connected (GpioPCA9555) !" << logs::end;
	else
		logger().debug() << "Hardware status: SwitchDriver OK" << logs::end;
}

SwitchDriver::~SwitchDriver()
{
}

bool SwitchDriver::is_connected()
{
	return GpioPCA9555::instance().isConnected();
}

int SwitchDriver::pressed(unsigned char pin)
{
	return GpioPCA9555::instance().getValueP1(pin);
}

int SwitchDriver::tirettePressed()
{
	return pressed(7);
}

int SwitchDriver::backLeftPressed()
{
	return pressed(0);
}

int SwitchDriver::backRightPressed()
{
	return pressed(1);
}

void SwitchDriver::setGPIO(int gpio, bool activate)
{
	if (activate)
		GpioPCA9555::instance().setOnP0(gpio);
	else
		GpioPCA9555::instance().setOffP0(gpio);
}
