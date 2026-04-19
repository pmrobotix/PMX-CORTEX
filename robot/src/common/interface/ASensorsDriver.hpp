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
 * \brief Miroir common des Settings expose par la balise Teensy (I2C 0x2D).
 *
 * Permet a MenuBeaconLCDTouch de lire/ecrire les champs de config
 * sans dependance directe sur BeaconSensors (driver-arm).
 * Les offsets I2C sont geres dans SensorsDriver (driver-arm).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md section 6 et
 * teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md struct Settings.
 */
struct MatchSettingsData
{
	// Bloc 1 : OPOS6UL -> Teensy (5 bytes)
	int8_t  numOfBots     = 3;
	int8_t  ledLuminosity = 10;
	uint8_t matchPoints   = 0;
	uint8_t matchState    = 0;   // Encode robot.phase() (0=CONFIG, 1=ARMED, 2=MATCH, 3=END)
	uint8_t lcdBacklight  = 1;
	// Bloc 2 : Teensy (LCD) -> OPOS6UL (5 bytes)
	uint8_t matchColor    = 0;
	uint8_t strategy      = 0;
	uint8_t testMode      = 0;
	uint8_t advDiameter   = 40;
	uint8_t actionReq     = 0;   // 1 = bouton SETPOS/RESET clique (sens selon matchState).
	                             // OPOS6UL remet a 0 apres consommation.
	// Bloc 3 : compteur de clic touch (1 byte)
	uint8_t seq_touch     = 0;
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
	 * \brief Sync complet init : write pending Settings + read Settings + read flag+getData.
	 * \return 1 si nouvelles donnees, 0 si pas de nouvelles donnees, -1 si erreur.
	 */
	virtual int syncFull() { return 0; }

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

	// ---- Settings balise (MatchSettingsData) ----
	// Default no-op pour SIMU. Override en driver-arm pour communiquer avec
	// la Teensy via I2C. Voir robot/md/O_STATE_NEW_INIT.md section 6.

	/*!
	 * \brief Lit le bloc Settings de la balise (0x2D).
	 * \return true si lecture OK, false si erreur I2C ou non supporte (SIMU).
	 */
	virtual bool readMatchSettings(MatchSettingsData& /*out*/) { return false; }

	/*!
	 * \brief Ecrit le champ matchColor (reg 5) sur la balise.
	 * \return true si ecriture OK, false sinon.
	 */
	virtual bool writeMatchColor(uint8_t /*c*/) { return false; }

	/*!
	 * \brief Ecrit le champ strategy (reg 6) sur la balise.
	 */
	virtual bool writeStrategy(uint8_t /*s*/) { return false; }

	/*!
	 * \brief Ecrit le champ advDiameter (reg 8) sur la balise.
	 */
	virtual bool writeAdvDiameter(uint8_t /*d*/) { return false; }

	/*!
	 * \brief Ecrit le champ matchState (reg 3) sur la balise.
	 */
	virtual bool writeMatchState(uint8_t /*s*/) { return false; }

	/*!
	 * \brief Ecrit le champ matchPoints (reg 2) sur la balise.
	 */
	virtual bool writeMatchPoints(uint8_t /*p*/) { return false; }

	/*!
	 * \brief Ecrit le champ numOfBots (reg 0) sur la balise.
	 */
	virtual bool writeNumOfBots(int8_t /*n*/) { return false; }

	/*!
	 * \brief Ecrit le champ actionReq (reg 9) sur la balise.
	 *        OPOS6UL l'utilise pour remettre actionReq a 0 apres consommation.
	 */
	virtual bool writeActionReq(uint8_t /*v*/) { return false; }

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
	 * \brief Positionne un adversaire persistant en SIMU.
	 *        Contrairement a addvPositionsAdv, cet adv reste present a chaque
	 *        sync() jusqu'a l'appel de clearInjectedAdv. Destine aux tests de
	 *        scenarios ou un adv reste plante pendant toute la duree du mouvement.
	 * \param x_table_mm Position X adversaire sur la table (repere table).
	 * \param y_table_mm Position Y adversaire sur la table.
	 * \note No-op sur les drivers ARM reels (l'adv vient de la balise physique).
	 */
	virtual void setInjectedAdv(float /*x_table_mm*/, float /*y_table_mm*/) {}

	/*!
	 * \brief Supprime l'adversaire persistant injecte.
	 * \note No-op sur les drivers ARM reels.
	 */
	virtual void clearInjectedAdv() {}

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
