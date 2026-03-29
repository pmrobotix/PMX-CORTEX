/*!
 * \file
 * \brief Implementation ARM du driver de boutons.
 */

#include "ButtonDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/ButtonDriver.hpp"

#include <unistd.h>
#include <cstdint>

AButtonDriver * AButtonDriver::create()
{
    if (!HardwareConfig::instance().isEnabled("ButtonDriver")) {
        return new ButtonDriverSimu();
    }
    return new ButtonDriver();
}

ButtonDriver::ButtonDriver()
{
}

ButtonDriver::~ButtonDriver()
{
}

bool ButtonDriver::pressed(ButtonTouch button)
{
    uint8_t adafruit_buttons = 0;
    adafruit_buttons = Adafruit_RGBLCDShield::instance().readButtons();

    if (adafruit_buttons) {
        switch (button) {
        case BUTTON_ENTER_KEY:
            return false;
            break;
        case BUTTON_BACK_KEY:
            return (adafruit_buttons & BUTTON_SELECT);
            break;
        case BUTTON_UP_KEY:
            return (adafruit_buttons & BUTTON_UP);
            break;
        case BUTTON_DOWN_KEY:
            return (adafruit_buttons & BUTTON_DOWN);
            break;
        case BUTTON_LEFT_KEY:
            return (adafruit_buttons & BUTTON_LEFT);
            break;
        case BUTTON_RIGHT_KEY:
            return (adafruit_buttons & BUTTON_RIGHT);
            break;
        case BUTTON_NONE:
            break;
        }

    }

    return 0;
}
