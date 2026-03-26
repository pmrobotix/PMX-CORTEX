/*!
 * \file
 * \brief Implementation ARM du driver d'ecran LCD pour l'OPOS6UL.
 *
 * Pilote l'ecran LCD 16x2 via le LCD Shield Adafruit (MCP23017 I2C).
 */

#ifndef OPOS6UL_LCDSHIELDDRIVER_HPP_
#define OPOS6UL_LCDSHIELDDRIVER_HPP_

#include <stddef.h>
#include <cstdint>

#include "interface/ALcdShieldDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation ARM du driver LCD.
 *
 * Utilise le LCD Shield Adafruit RGB 16x2 connecte via le MCP23017 I2C.
 */
class LcdShieldDriver: public ALcdShieldDriver
{
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("LcdShieldDriver.OPO");
		return instance;
	}

	bool connected_;  ///< true si le LCD est connecte en I2C.

public:

	/*!
	 * \brief Constructeur. Initialise le LCD 16x2.
	 */
	LcdShieldDriver();

	/*!
	 * \brief Destructeur.
	 */
	~LcdShieldDriver();

	bool is_connected() override;
	void clear() override;
	void home() override;
	void setBacklightOn() override;
	void setBacklightOff() override;
	void setCursor(uint8_t, uint8_t) override;
	size_t write(uint8_t value) override;
	void print_content_string(std::string str, int row, int col = 1) override;
	void print_content_integer(int value, int row, int col = 1) override;

};

#endif
