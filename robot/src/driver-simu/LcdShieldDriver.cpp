/*!
 * \file
 * \brief Implementation SIMU du driver LCD.
 */

#include "LcdShieldDriver.hpp"

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
    logger().info() << "setBacklightOn()" << logs::end;
}

void LcdShieldDriverSimu::setBacklightOff()
{
    logger().info() << "setBacklightOff()" << logs::end;
}

void LcdShieldDriverSimu::setCursor(uint8_t col, uint8_t row)
{

}

void LcdShieldDriverSimu::print_content_string(std::string str, int row, int col)
{
    logger().info() << str << logs::end;
}

void LcdShieldDriverSimu::print_content_integer(int value, int row, int col)
{
    logger().info() << value << logs::end;
}
