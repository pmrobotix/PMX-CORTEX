/*!
 * \file
 * \brief Implementation ARM du driver de LEDs pour l'OPOS6UL.
 */

#include "LedDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/LedDriver.hpp"

#include <unistd.h>
#include <string>

#include "log/Logger.hpp"

ALedDriver* ALedDriver::create(std::string, int nb)
{
	if (!HardwareConfig::instance().isEnabled("LedDriver")) {
		return new LedDriverSimu(nb);
	}
	return new LedDriver(nb);
}

LedDriver::LedDriver(int nb)
{
	// OPOS6UL : GPIO_num = (Bank - 1) * 32 + Pin
	// GPIO5_8 = 136, GPIO5_7 = 135, ..., GPIO5_1 = 129
	nb_ = 8;

	gpio[0] = new AsGpio(136);
	gpio[1] = new AsGpio(135);
	gpio[2] = new AsGpio(134);
	gpio[3] = new AsGpio(133);
	gpio[4] = new AsGpio(132);
	gpio[5] = new AsGpio(129);
	gpio[6] = new AsGpio(130);
	gpio[7] = new AsGpio(131);

	for (int i = 0; i < 8; i++) {
		if (gpio[i] != NULL) {
			gpio[i]->setPinDirection((char*) "out");
			gpio[i]->setIrqMode((char*) "none");
			gpio[i]->setPinValue(0);
		}
	}
}

LedDriver::~LedDriver()
{
}

void LedDriver::setBit(int index, LedColor color)
{
	int c = 0;
	if (color != LED_OFF)
		c = 1;
	gpio[index]->setPinValue(c);
}

void LedDriver::setBytes(uint hex, LedColor color)
{
	for (int i = 0; i < nb_; i++) {
		if (((hex >> i) & 0x01) == 1) {
			setBit(i, color);
		} else {
			setBit(i, LED_OFF);
		}
	}
}

int LedDriver::getBit(int)
{
	return -1; // pas de lecture hardware sur GPIO
}
