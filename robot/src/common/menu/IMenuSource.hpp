/*!
 * \file
 * \brief Interface abstraite pour les sources d'input/output du menu d'init.
 *
 * Voir robot/md/O_STATE_NEW_INIT.md pour le contexte complet.
 *
 * Chaque source (MenuShieldLCD, MenuBeaconLCDTouch, futurs...) implemente :
 *   - pollInputs : lit ses propres inputs et ecrit dans Robot via les setters.
 *   - refreshDisplay : lit l'etat Robot et le pousse vers son support d'affichage.
 *
 * Le MenuController garantit l'ordre "tous les poll avant tous les refresh"
 * pour eviter les desynchros entre sources.
 */

#ifndef COMMON_MENU_IMENUSOURCE_HPP_
#define COMMON_MENU_IMENUSOURCE_HPP_

class Robot;

class IMenuSource
{
public:
	virtual ~IMenuSource() = default;

	/*!
	 * \brief Lit les inputs de la source (boutons, I2C, touch...) et met a jour Robot
	 *        via les setters phase-aware.
	 *
	 * Seule voie d'ecriture vers Robot depuis cette source. Ne doit JAMAIS appeler
	 * ses propres methodes d'affichage (c'est le role de refreshDisplay, appele ensuite
	 * par le controller).
	 */
	virtual void pollInputs(Robot& robot) = 0;

	/*!
	 * \brief Rafraichit l'affichage de la source a partir de l'etat courant de Robot.
	 *
	 * Ne doit JAMAIS appeler les setters de Robot. Lit uniquement via les getters.
	 * Pour les sources cross-MCU (ex: balise), push l'etat Robot vers le medium
	 * (ex: ecriture I2C Settings).
	 */
	virtual void refreshDisplay(const Robot& robot) = 0;

	/*!
	 * \brief Indique si la source est encore vivante (hardware repond).
	 *        Le controller saute pollInputs/refreshDisplay sur les sources mortes.
	 */
	virtual bool isAlive() const = 0;

	/*!
	 * \brief Nom de la source pour les logs.
	 */
	virtual const char* name() const = 0;
};

#endif
