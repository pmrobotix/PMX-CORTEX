/*!
 * \file
 * \brief Implementation SIMU du driver de switchs et tirette.
 */

#include "SwitchDriver.hpp"

#include <chrono>
#include <thread>

SwitchDriverSimu::SwitchDriverSimu()
{
	state_ = 1;
}

SwitchDriverSimu::~SwitchDriverSimu()
{
}

bool SwitchDriverSimu::is_connected()
{
	return true;
}

int SwitchDriverSimu::tirettePressed()
{
	std::this_thread::sleep_for(std::chrono::microseconds(250000));
	return 0;
}

int SwitchDriverSimu::backLeftPressed()
{
	return 0;
}

int SwitchDriverSimu::backRightPressed()
{
	return 0;
}

void SwitchDriverSimu::setGPIO(int, bool)
{
}
