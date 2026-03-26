/*!
 * \file
 * \brief Implementation SIMU de la position partagee du robot.
 *
 * Acces thread-safe a la position du robot via mutex.
 */

#ifndef SIMU_ROBOTPOSITIONSHARED_HPP_
#define SIMU_ROBOTPOSITIONSHARED_HPP_

#include "interface/ARobotPositionShared.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation de la position partagee.
 *
 * Singleton thread-safe, identique a la version ARM.
 */
class RobotPositionShared: public ARobotPositionShared, utils::Mutex
{
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("RobotPositionShared.SIMU");
		return instance;
	}

	ROBOTPOSITION p_;  ///< Position courante du robot.

	unsigned long long t_set_us_;  ///< Timestamp du dernier set (microsecondes).

public:

	/*!
	 * \brief Constructeur.
	 */
	RobotPositionShared();

	/*!
	 * \brief Destructeur.
	 */
	~RobotPositionShared();

	ROBOTPOSITION getRobotPosition(int debug = 0) override;

	void setRobotPosition(ROBOTPOSITION p) override;

};

#endif
