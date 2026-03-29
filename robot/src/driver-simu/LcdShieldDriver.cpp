/*!
 * \file
 * \brief Implementation SIMU du driver LCD.
 */

#include "LcdShieldDriver.hpp"

#include <stddef.h>
#include <cstdint>
#include <string>

LcdShieldDriverSimu::LcdShieldDriverSimu()
{

}

LcdShieldDriverSimu::~LcdShieldDriverSimu()
{
}

bool LcdShieldDriverSimu::is_connected()
{
    return true;
}

void LcdShieldDriverSimu::clear()
{

}

void LcdShieldDriverSimu::home()
{

}

void LcdShieldDriverSimu::setBacklightOn()
{
    logger().debug() << "setBacklightOn()" << logs::end;
}

void LcdShieldDriverSimu::setBacklightOff()
{
    logger().debug() << "setBacklightOff()" << logs::end;
}

void LcdShieldDriverSimu::setCursor(uint8_t col, uint8_t row)
{

}

size_t LcdShieldDriverSimu::write(uint8_t value)
{
    logger().debug() << value << logs::end;
    return 1;
}

void LcdShieldDriverSimu::print_content_string(std::string str, int row, int col)
{
    logger().debug() << str << logs::end;
}

void LcdShieldDriverSimu::print_content_integer(int value, int row, int col)
{
    logger().debug() << value << logs::end;
}
