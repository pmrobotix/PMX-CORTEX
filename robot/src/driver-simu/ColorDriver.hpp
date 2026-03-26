/*!
 * \file
 * \brief Implementation SIMU du driver de capteur couleur.
 *
 * Stub qui retourne des valeurs par defaut.
 */

#ifndef SIMU_COLORDRIVER_HPP_
#define SIMU_COLORDRIVER_HPP_

#include "interface/AColorDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver couleur.
 *
 * Retourne des valeurs neutres (pas de capteur en simulation).
 */
class ColorDriver: public AColorDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ColorDriver.SIMU");
		return instance;
	}

public:

	/*!
	 * \brief Constructeur.
	 */
	ColorDriver();

	/*!
	 * \brief Destructeur.
	 */
	~ColorDriver();

	bool readRGB() override;
	float getTX() override;
	float getTY() override;

};

#endif
