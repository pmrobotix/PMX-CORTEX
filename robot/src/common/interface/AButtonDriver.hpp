/*!
 * \file
 * \brief Interface abstraite du driver de boutons.
 *
 * Gere les boutons physiques du robot (navigation dans les menus,
 * selection de tests fonctionnels).
 */

#ifndef ABUTTONDRIVER_HPP_
#define ABUTTONDRIVER_HPP_

/*!
 * \brief Enumeration des boutons disponibles.
 */
enum ButtonTouch
{
	BUTTON_ENTER_KEY,  ///< Touche Entree/Valider.
	BUTTON_BACK_KEY,   ///< Touche Retour.
	BUTTON_UP_KEY,     ///< Touche Haut.
	BUTTON_DOWN_KEY,   ///< Touche Bas.
	BUTTON_LEFT_KEY,   ///< Touche Gauche.
	BUTTON_RIGHT_KEY,  ///< Touche Droite.
	BUTTON_NONE        ///< Sentinelle de fin d'enumeration / aucun bouton.
};

/*!
 * \brief Interface abstraite du driver de boutons.
 *
 * Fournit la lecture de l'etat des boutons physiques.
 * L'implementation ARM lit les GPIO, la SIMU lit le clavier.
 */
class AButtonDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \return Pointeur vers l'instance creee.
	 */
	static AButtonDriver * create();

	/*!
	 * \brief Teste si un bouton est actuellement appuye.
	 * \param button Bouton a tester.
	 * \return true si le bouton est appuye.
	 */
	virtual bool pressed(ButtonTouch button) = 0;

	virtual ~AButtonDriver()
	{
	}

protected:

	AButtonDriver()
	{
	}

};

#endif
