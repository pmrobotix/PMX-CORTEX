/*!
 * \file
 * \brief Implementation ARM du driver de LEDs pour l'OPOS6UL.
 *
 * Pilote 8 LEDs connectees sur les GPIO5_1 a GPIO5_8
 * via le SDK Armadeus (as_gpio).
 */

#ifndef OPOS6UL_LEDDRIVER_HPP_
#define OPOS6UL_LEDDRIVER_HPP_

#include <sys/types.h>

#include "interface/ALedDriver.hpp"
#include "log/LoggerFactory.hpp"

#include <as_devices/cpp/as_gpio.hpp>

/*!
 * \brief Implementation ARM du driver de LEDs.
 *
 * Chaque LED est controlee par un GPIO de l'OPOS6UL.
 * Les numeros GPIO sont calcules selon la formule Armadeus :
 * GPIO_num = (Bank - 1) * 32 + Pin.
 */
class LedDriver: public ALedDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("LedDriver.OPO");
		return instance;
	}

	int nb_;  ///< Nombre de LEDs.

public:

	AsGpio * gpio[8];  ///< Tableau de pointeurs vers les GPIO Armadeus.

	/*!
	 * \brief Constructeur. Initialise les 8 GPIO en sortie.
	 * \param nb Nombre de LEDs (force a 8).
	 */
	LedDriver(int nb);

	/*!
	 * \brief Destructeur.
	 */
	~LedDriver();

	void setBit(int index, LedColor color) override;
	void setBytes(uint hex, LedColor color) override;
	int getBit(int index) override;
};

#endif
