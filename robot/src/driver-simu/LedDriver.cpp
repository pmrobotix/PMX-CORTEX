/*!
 * \file
 * \brief Implementation SIMU du driver de LEDs.
 */

#include "LedDriver.hpp"

#include <sstream>
#include <string>

#include "log/Logger.hpp"

LedDriverSimu::LedDriverSimu(int nb)
{
	hexa = 0;
	nb_ = nb;
	gpio = new int[nb_];

	for (int i = 0; i < nb_; i++) {
		gpio[i] = 0;
	}
}

LedDriverSimu::~LedDriverSimu()
{
	delete[] gpio;
}

void LedDriverSimu::setBit(int index, LedColor color)
{
	gpio[index] = color;

	hexa ^= (-1 ^ hexa) & (1 << index);

	std::ostringstream ost;
	ost << "LED ";
	for (int i = 0; i < nb_; i++) {
		if (i == index)
			ost << "\033[1m" << "\033[4;31m" << gpio[i] << "\033[0m";
		else
			ost << "\033[0m" << gpio[i];
	}
	ost << " (POS=" << index << ")";
	logger().debug() << ost.str() << logs::end;
}

void LedDriverSimu::setBytes(uint hex, LedColor color)
{
	hexa = hex;

	for (int i = 0; i < nb_; i++) {
		if (((hex >> i) & 0x01) == 1) {
			gpio[i] = color;
		} else {
			gpio[i] = 0;
		}
	}

	std::ostringstream ost;
	ost << "LED ";
	for (int i = 0; i < nb_; i++) {
		ost << gpio[i];
	}
	logger().debug() << ost.str() << logs::end;
}

int LedDriverSimu::getBit(int index)
{
	return gpio[index];
}
