/*!
 * \file
 * \brief Implementation SIMU du driver de LEDs.
 *
 * Simule une barre de LEDs en affichant leur etat dans la console.
 */

#ifndef SIMU_LEDDRIVER_HPP_
#define SIMU_LEDDRIVER_HPP_

#include <sys/types.h>

#include "interface/ALedDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver de LEDs.
 *
 * Stocke l'etat des LEDs dans un tableau et affiche
 * les changements dans la console via le logger.
 */
class LedDriverSimu: public ALedDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("LedDriver.SIMU");
		return instance;
	}

	int nb_;   ///< Nombre de LEDs.

public:

	int *gpio;  ///< Etat de chaque LED (couleur).
	uint hexa;  ///< Masque binaire de l'etat de la barre.

	/*!
	 * \brief Constructeur.
	 * \param nb Nombre de LEDs dans la barre.
	 */
	LedDriverSimu(int nb);

	/*!
	 * \brief Destructeur. Libere le tableau gpio.
	 */
	~LedDriverSimu();

	void setBit(int index, LedColor color) override;
	void setBytes(uint hex, LedColor color) override;
	int getBit(int index) override;
};

#endif
