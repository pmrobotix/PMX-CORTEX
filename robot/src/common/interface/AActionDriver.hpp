/*!
 * \file
 * \brief Interface abstraite du driver d'actions specifiques au robot.
 *
 * Definit les actions hardware specifiques a chaque annee de competition
 * (bras, pinces, systemes de collecte, etc.).
 */

#ifndef AACTIONDRIVER_HPP_
#define AACTIONDRIVER_HPP_

/*!
 * \brief Interface abstraite du driver d'actions.
 *
 * Classe de base pour les actions hardware specifiques au robot.
 * Chaque annee, l'implementation concrete definit les actions
 * propres aux mecanismes du robot.
 */
class AActionDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param nb Nombre d'actions.
	 * \return Pointeur vers l'instance creee.
	 */
	static AActionDriver * create(int nb);

	int example1;  ///< Exemple de variable d'action.

#ifdef SIMU
	int exampleSimu;  ///< Variable specifique a la simulation.
#endif

	/*!
	 * \brief Execute une action.
	 * \param value Parametre de l'action.
	 */
	virtual void function(int value);

	virtual ~AActionDriver()
	{
	}

protected:

	AActionDriver()
	{
		example1 = 0;
#ifdef SIMU
		exampleSimu = 0;
#endif
	}

};

#endif
