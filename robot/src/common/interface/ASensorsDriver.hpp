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

#include <cstdint>
#include <string>
#include <vector>

class ARobotPositionShared;
class Robot;

/*!
 * \brief Position d'un robot adversaire detecte par la balise.
 *
 * Inclut le delta temps de mesure beacon (t_us) pour la synchronisation
 * avec le buffer circulaire de positions dans ARobotPositionShared.
 */
class RobotPos
{
public:
	int nbDetectedBots;  ///< Nombre total de robots detectes (duplique dans chaque element).
	float x;             ///< Position X de l'adversaire en mm (coordonnees table).
	float y;             ///< Position Y de l'adversaire en mm (coordonnees table).
	float theta_deg;     ///< Angle de l'adversaire par rapport a l'avant du robot en degres.
	float d;             ///< Distance du centre robot au centre adversaire en mm.
	uint16_t t_us;       ///< Delta temps de mesure beacon (us depuis debut cycle Teensy). 0 en SIMU.

	/*!
	 * \brief Constructeur.
	 * \param nb Nombre de robots detectes.
	 * \param x_ Position X en mm.
	 * \param y_ Position Y en mm.
	 * \param a_ Angle en degres.
	 * \param d_ Distance en mm.
	 * \param t_us_ Delta temps mesure beacon en us (0 si inconnu/SIMU).
	 */
	RobotPos(int nb, float x_, float y_, float a_, float d_, uint16_t t_us_ = 0)
	{
		nbDetectedBots = nb;
		x = x_;
		y = y_;
		theta_deg = a_;
		d = d_;
		t_us = t_us_;
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

	/*!
	 * \brief Ecrit la luminosite de la matrice LED de la balise beacon.
	 * \param lum Luminosite 0..100.
	 */
	virtual void writeLedLuminosity(uint8_t lum) = 0;

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

	/*!
	 * \brief Retourne le numero de sequence beacon (debug).
	 * \return 0 en SIMU, incremente chaque cycle en ARM.
	 */
	virtual uint32_t getBeaconSeq() { return 0; }

	/*!
	 * \brief Retourne le timestamp (ms) du dernier sync I2C reussi.
	 * \return Timestamp depuis le chrono partage. Plus precis que mesurer "avant sync()"
	 *         car capture juste apres la detection de la nouvelle donnee beacon.
	 *         0 en SIMU.
	 */
	virtual uint32_t getLastSyncMs() { return 0; }

	virtual ~ASensorsDriver()
	{
	}

protected:

	ASensorsDriver()
	{
	}

};

#endif
