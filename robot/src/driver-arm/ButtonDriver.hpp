/*!
 * \file
 * \brief Implementation ARM du driver de boutons pour l'OPOS6UL.
 *
 * Lit les boutons via le LCD Shield Adafruit (MCP23017 I2C).
 */

#ifndef OPOS6UL_BUTTONDRIVER_HPP_
#define OPOS6UL_BUTTONDRIVER_HPP_

#include "interface/AButtonDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "Adafruit_RGBLCDShield.hpp"

/*!
 * \brief Implementation ARM du driver de boutons.
 *
 * Les boutons physiques sont lus via le GPIO expander MCP23017
 * du LCD Shield Adafruit connecte en I2C.
 */
class ButtonDriver: public AButtonDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ButtonDriver.OPO");
		return instance;
	}

public:

	/*!
	 * \brief Constructeur.
	 */
	ButtonDriver();

	/*!
	 * \brief Destructeur.
	 */
	~ButtonDriver();

	bool pressed(ButtonTouch button) override;

};

#endif
