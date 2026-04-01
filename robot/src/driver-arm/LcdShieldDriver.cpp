/*!
 * \file
 * \brief Implementation ARM du driver LCD.
 */

#include "LcdShieldDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/LcdShieldDriver.hpp"

#include <stddef.h>
#include <cstdint>
#include <string>

#include "Adafruit_RGBLCDShield.hpp"

ALcdShieldDriver * ALcdShieldDriver::create(std::string botId)
{
    if (!HardwareConfig::instance().isEnabled("LcdShieldDriver")) {
        return new LcdShieldDriverSimu();
    }
    return new LcdShieldDriver();
}

LcdShieldDriver::LcdShieldDriver()
{
    connected_ = 0;
    //configuration LCD 16 colonnes x 2 lignes
    connected_ = Adafruit_RGBLCDShield::instance().begin(16, 2);
    Adafruit_RGBLCDShield::instance().clear();
}

LcdShieldDriver::~LcdShieldDriver()
{
}

bool LcdShieldDriver::is_connected()
{
    return connected_;
}

void LcdShieldDriver::clear()
{
    Adafruit_RGBLCDShield::instance().clear();
}

void LcdShieldDriver::home()
{
    Adafruit_RGBLCDShield::instance().home();
}

void LcdShieldDriver::setBacklightOn()
{
    Adafruit_RGBLCDShield::instance().setBacklight(LCD_ON);
}

void LcdShieldDriver::setBacklightOff()
{
    Adafruit_RGBLCDShield::instance().setBacklight(LCD_OFF);
}

void LcdShieldDriver::setCursor(uint8_t col, uint8_t row)
{
    Adafruit_RGBLCDShield::instance().setCursor(col, row);
}

void LcdShieldDriver::print_content_string(std::string str, int row, int col)
{
    for (char c : str)
    {
        Adafruit_RGBLCDShield::instance().write__(static_cast<uint8_t>(c));
    }
}

void LcdShieldDriver::print_content_integer(int value, int row, int col)
{
    print_content_string(std::to_string(value), row, col);
}
