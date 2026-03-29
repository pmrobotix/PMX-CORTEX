/*!
 * \file
 * \brief Implementation SIMU du driver de switchs et tirette.
 *
 * Simule la tirette et les switchs arriere. La tirette retourne
 * toujours 0 (retiree) apres un delai, les switchs retournent 0.
 */

#ifndef SIMU_SWITCHDRIVER_HPP_
#define SIMU_SWITCHDRIVER_HPP_

#include "interface/ASwitchDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver de switchs.
 */
class SwitchDriverSimu: public ASwitchDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("SwitchDriver.SIMU");
		return instance;
	}

public:

	int state_;  ///< Etat interne (non utilise en SIMU).

	/*!
	 * \brief Constructeur.
	 */
	SwitchDriverSimu();

	/*!
	 * \brief Destructeur.
	 */
	~SwitchDriverSimu();

	bool is_connected() override;
	int tirettePressed() override;
	int backLeftPressed() override;
	int backRightPressed() override;
	void setGPIO(int gpio, bool activate) override;
};

#endif
