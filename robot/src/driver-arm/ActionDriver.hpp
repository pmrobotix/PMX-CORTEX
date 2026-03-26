/*!
 * \file
 * \brief Implementation ARM du driver d'actions pour l'OPOS6UL.
 *
 * Actions hardware specifiques au robot de competition.
 */

#ifndef OPOS6UL_ACTIONDRIVER_HPP_
#define OPOS6UL_ACTIONDRIVER_HPP_

#include "interface/AActionDriver.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Implementation ARM du driver d'actions.
 *
 * Definit les actions propres aux mecanismes du robot pour
 * l'annee de competition en cours.
 */
class ActionDriver: public AActionDriver
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("ActionDriver.OPO");
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
