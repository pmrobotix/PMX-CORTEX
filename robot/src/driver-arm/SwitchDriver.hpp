/*!
 * \file
 * \brief Implementation ARM du driver de switchs pour l'OPOS6UL.
 *
 * Lit la tirette et les switchs arriere via le GPIO expander
 * PCA9555 connecte en I2C. Controle aussi les sorties GPIO.
 */

#ifndef OPOS6UL_SWITCHDRIVER_HPP_
#define OPOS6UL_SWITCHDRIVER_HPP_

#include <string>

#include "interface/ASwitchDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation ARM du driver de switchs.
 *
 * Utilise le PCA9555 (port1 = entrees) pour lire la tirette (pin 7)
 * et les switchs arriere (pins 0-1). Le port0 sert de sorties GPIO.
 */
class SwitchDriver: public ASwitchDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SwitchDriver.OPO");
		return instance;
	}

	/*!
	 * \brief Lit l'etat d'un pin du port1 du PCA9555.
	 * \param pin Numero du pin (0-7).
	 * \return 0 ou 1.
	 */
	int pressed(unsigned char pin);

public:

	/*!
	 * \brief Constructeur. Initialise le PCA9555.
	 */
	SwitchDriver();

	/*!
	 * \brief Destructeur.
	 */
	~SwitchDriver();

	bool is_connected() override;
	int tirettePressed() override;
	int backLeftPressed() override;
	int backRightPressed() override;
	void setGPIO(int gpio, bool activate) override;
};

#endif
