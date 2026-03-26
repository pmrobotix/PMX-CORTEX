/*!
 * \file
 * \brief Implementation SIMU du driver de capteur couleur.
 */

#include "ColorDriver.hpp"

AColorDriver * AColorDriver::create(std::string botName)
{
    return new ColorDriver();
}

ColorDriver::ColorDriver()
{
}

ColorDriver::~ColorDriver()
{
}

bool ColorDriver::readRGB()
{
    return true;
}

float ColorDriver::getTX()
{
    return 0.0f;
}

float ColorDriver::getTY()
{
    return 0.0f;
}
