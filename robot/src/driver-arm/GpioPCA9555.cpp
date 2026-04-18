/*!
 * \file
 * \brief Implementation du driver I2C PCA9555.
 */

#include "GpioPCA9555.hpp"

#include <unistd.h>

#include "log/Logger.hpp"

GpioPCA9555::GpioPCA9555() :
		i2c_(1, GPIOBOARD_PCA9555), connected_(false), port0Value_(0), port1Value_(0)
{
}

bool GpioPCA9555::begin()
{
	if (!i2c_.open()) return false;
	return setup();
}

bool GpioPCA9555::setup()
{
	long r = write_i2c(CONFIG_P0, 0x00); // Port0 = sorties
	if (r < 0) {
		connected_ = false;
		logger().error() << "Hardware status: GpioPCA9555 is NOT connected !" << logs::end;
		return connected_;
	} else {
		connected_ = true;
		logger().debug() << "Hardware status: GpioPCA9555 OK" << logs::end;
		write_i2c(OUT_P0, 0x00);          // RAZ sorties
		write_i2c(CONFIG_P1, 0xFF);       // Port1 = entrees
		write_i2c(IN_P1, 0x00);
		utils::sleep_for_micros(PAUSE);
	}
	return connected_;
}

void GpioPCA9555::setValueP0(int port, int pin, int value)
{
	if (!connected_) {
		logger().error() << "setValueP0() : GpioBoard NOT CONNECTED !" << logs::end;
		return;
	}

	int out = 0;

	if (value == 1)
		out = port0Value_ | (0x01 << pin);
	else if (value == 0)
		out = port0Value_ & (0xFE << pin);

	write_i2c(port, out);
}

void GpioPCA9555::setOnP0(int pin)
{
	setValueP0(OUT_P0, pin, 1);
}

void GpioPCA9555::setOffP0(int pin)
{
	setValueP0(OUT_P0, pin, 0);
}

int GpioPCA9555::getValueP1(int pin)
{
	if (!connected_) {
		logger().error() << "getValueP1() : return 0; GpioBoard NOT CONNECTED !" << logs::end;
		return -1;
	}
	unsigned char in = read_i2c(IN_P1);
	logger().debug() << "getValueP1 in = " << reinterpret_cast<void*>(in) << logs::end;
	int intmp = (in >> pin) & 0x01;
	logger().debug() << "getValueP1 in" << pin << "=" << reinterpret_cast<void*>(intmp) << logs::end;
	return intmp;
}

long GpioPCA9555::write_i2c(unsigned char command, unsigned char value)
{
	return i2c_.writeReg(command, &value, 1);
}

long GpioPCA9555::read_i2c(unsigned char command)
{
	return i2c_.readRegByte(command);
}
