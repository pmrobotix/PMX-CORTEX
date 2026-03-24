/*!
 * \file
 * \brief Implementation SIMU du driver de switchs et tirette.
 */

#include "SwitchDriver.hpp"

#include <chrono>
#include <thread>

ASwitchDriver * ASwitchDriver::create(std::string)
{
	static SwitchDriver *instance = new SwitchDriver();
	return instance;
}

SwitchDriver::SwitchDriver()
{
	state_ = 1;
}

SwitchDriver::~SwitchDriver()
{
}

bool SwitchDriver::is_connected()
{
	return true;
}

int SwitchDriver::tirettePressed()
{
	std::this_thread::sleep_for(std::chrono::microseconds(250000));
	return 0;
}

int SwitchDriver::backLeftPressed()
{
	return 0;
}

int SwitchDriver::backRightPressed()
{
	return 0;
}

void SwitchDriver::setGPIO(int, bool)
{
}
