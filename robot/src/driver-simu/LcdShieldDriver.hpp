/*!
 * \file
 * \brief Implementation SIMU du driver d'ecran LCD.
 *
 * Stub qui log les commandes LCD sans hardware reel.
 */

#ifndef SIMU_LCDSHIELDDRIVER_HPP_
#define SIMU_LCDSHIELDDRIVER_HPP_

#include <stddef.h>
#include <cstdint>

#include "interface/ALcdShieldDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver LCD.
 *
 * Toutes les methodes sont des stubs qui loggent les appels.
 */
class LcdShieldDriver: public ALcdShieldDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("LcdShieldDriver.SIMU");
		return instance;
	}

public:

	/*!
	 * \brief Constructeur.
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
