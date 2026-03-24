/*!
 * \file
 * \brief Interface abstraite du driver de capteurs (balise ToF, IR, ultrasons).
 *
 * Definit l'acces aux capteurs de detection d'obstacles :
 * balise ToF (positions adversaires), capteurs de proximite
 * (avant/arriere/lateraux) et affichage 7 segments.
 */

#ifndef ASENSORSDRIVER_HPP_
#define ASENSORSDRIVER_HPP_

#include <string>
#include <vector>

class ARobotPositionShared;
class Robot;

/*!
 * \brief Position d'un robot adversaire detecte par la balise.
 */
class RobotPos
{
public:
	int nbDetectedBots;  ///< Nombre total de robots detectes (duplique dans chaque element).
	float x;             ///< Position X de l'adversaire en mm (coordonnees table).
	float y;             ///< Position Y de l'adversaire en mm (coordonnees table).
	float theta_deg;     ///< Angle de l'adversaire par rapport a l'avant du robot en degres.
	float d;             ///< Distance du centre robot au centre adversaire en mm.

	/*!
	 * \brief Constructeur.
	 * \param nb Nombre de robots detectes.
	 * \param x_ Position X en mm.
	 * \param y_ Position Y en mm.
	 * \param a_ Angle en degres.
	 * \param d_ Distance en mm.
	 */
	RobotPos(int nb, float x_, float y_, float a_, float d_)
	{
		nbDetectedBots = nb;
		x = x_;
		y = y_;
		theta_deg = a_;
		d = d_;
	}
};

/*!
 * \brief Interface abstraite du driver de capteurs.
 *
 * Fournit l'acces aux donnees des capteurs de detection d'obstacles.
 * L'implementation ARM lit les capteurs I2C reels,
 * l'implementation SIMU fournit des positions injectees manuellement.
 */
class ASensorsDriver {

public:

	typedef std::vector<RobotPos> bot_positions;  ///< Liste de positions adversaires detectees.

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botName Nom du robot.
	 * \param aRobotPositionShared Pointeur vers la position partagee du robot.
	 * \return Pointeur vers l'instance creee.
	 */
	static ASensorsDriver* create(std::string botName, ARobotPositionShared *aRobotPositionShared);

	/*!
	 * \brief Verifie si la balise est connectee.
	 * \return true si la communication est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Synchronise les donnees de la balise (lecture I2C).
	 * \return 0 si succes, -1 si erreur.
	 */
	virtual int sync() = 0;

	/*!
	 * \brief Retourne les positions des adversaires detectes.
	 * \return Vecteur de positions adversaires.
	 */
	virtual bot_positions getvPositionsAdv() = 0;

	// ---- Capteurs de proximite ----

	/*!
	 * \brief Lit le capteur avant gauche.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int frontLeft() = 0;

	/*!
	 * \brief Lit le capteur avant centre.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int frontCenter() = 0;

	/*!
	 * \brief Lit le capteur avant droit.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int frontRight() = 0;

	/*!
	 * \brief Lit le capteur arriere gauche.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int backLeft() = 0;

	/*!
	 * \brief Lit le capteur arriere centre.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int backCenter() = 0;

	/*!
	 * \brief Lit le capteur arriere droit.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int backRight() = 0;

	/*!
	 * \brief Lit le capteur lateral droit.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int rightSide() = 0;

	/*!
	 * \brief Lit le capteur lateral gauche.
	 * \return Distance en mm (0 si rien detecte).
	 */
	virtual int leftSide() = 0;

	/*!
	 * \brief Affiche un nombre sur l'afficheur 7 segments.
	 * \param number Nombre a afficher.
	 */
	virtual void displayNumber(int number) = 0;

	// ---- Simulation uniquement ----

	/*!
	 * \brief Ajoute une position adversaire simulee.
	 * \param x Position X en mm.
	 * \param y Position Y en mm.
	 * \note Utilise uniquement en mode SIMU.
	 */
	virtual void addvPositionsAdv(float x, float y) = 0;

	/*!
	 * \brief Supprime toutes les positions adversaires simulees.
	 * \note Utilise uniquement en mode SIMU.
	 */
	virtual void clearPositionsAdv() = 0;

	virtual ~ASensorsDriver()
	{
	}

protected:

	ASensorsDriver()
	{
	}

};

#endif
