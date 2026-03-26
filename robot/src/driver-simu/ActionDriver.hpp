/*!
 * \file
 * \brief Implementation SIMU du driver d'actions.
 *
 * Stub pour la simulation.
 */

#ifndef SIMU_ACTIONDRIVER_HPP_
#define SIMU_ACTIONDRIVER_HPP_

#include "interface/AActionDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation simulation du driver d'actions.
 *
 * Stub : les actions n'ont pas d'effet en simulation.
 */
class ActionDriver: public AActionDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ActionDriver.SIMU");
		return instance;
	}

public:

	/*!
	 * \brief Constructeur.
	 * \param nb Nombre d'actions.
	 */
	ActionDriver(int nb);

	/*!
	 * \brief Destructeur.
	 */
	~ActionDriver();

};

#endif
