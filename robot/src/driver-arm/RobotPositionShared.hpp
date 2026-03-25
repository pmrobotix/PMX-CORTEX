/*!
 * \file
 * \brief Implementation ARM de la position partagee du robot.
 *
 * Acces thread-safe a la position du robot via mutex.
 */

#ifndef OPOS6UL_ROBOTPOSITIONSHARED_HPP_
#define OPOS6UL_ROBOTPOSITIONSHARED_HPP_

#include "interface/ARobotPositionShared.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation ARM de la position partagee.
 *
 * Singleton thread-safe. L'asservissement ecrit la position,
 * les capteurs et la strategie la lisent.
 */
class RobotPositionShared: public ARobotPositionShared, utils::Mutex
{
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("RobotPositionShared.OPO");
		return instance;
	}

	ROBOTPOSITION p_;

	unsigned long long t_set_us_;

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
