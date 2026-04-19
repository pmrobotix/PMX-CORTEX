#ifndef COMMON_ROBOT_HPP_
#define COMMON_ROBOT_HPP_

#include <stdio.h>
#include <atomic>
#include <cstdint>
#include <string>

#include "log/LoggerFactory.hpp"
#include "utils/Arguments.hpp"
#include "interface/AAsservDriver.hpp"
#include "utils/ConsoleManager.hpp"
#include "state/Automate.hpp"
#include "utils/Chronometer.hpp"

class SvgWriter;

class Asserv;

class Actions;
class Arguments;
class ConsoleManager;
class TableGeometry;
class Sensors;

/*!
 * \brief Couleur de l'équipe du robot sur la table.
 */
enum RobotColor {
	PMXNOCOLOR, PMXYELLOW, PMXBLUE
};

/*!
 * \brief Phase du match utilisée par O_State_NewInit et MenuController.
 *
 * Gouverne les règles d'édition sur Robot : quels champs peuvent être
 * modifiés à quel moment. Voir robot/md/O_STATE_NEW_INIT.md.
 *
 * Transitions :
 *   CONFIG -> ARMED  : setPos trigger (bouton touch "SETPOS" ou BACK shield).
 *                      setPos() est execute, robot place, couleur verrouillee.
 *   ARMED  -> CONFIG : reset trigger (bouton touch "RESET" ou BACK shield).
 *                      freeMotion, couleur de nouveau editable.
 *   ARMED  -> MATCH  : edge "tirette inseree puis retiree".
 *   MATCH  -> END    : fin des 90s.
 */
enum MatchPhase {
	PHASE_CONFIG = 0,  ///< Menu ouvert, tout editable (y compris couleur)
	PHASE_ARMED  = 1,  ///< setPos fait. Couleur LOCKED. Strat/diam/LED/test editables.
	PHASE_MATCH  = 2,  ///< Tirette retiree, match en cours
	PHASE_END    = 3,  ///< Fin match
};

/*!
 * \brief Classe générique de base du robot.
 *
 * Robot fournit l'infrastructure commune à tous les robots du projet :
 * asservissement, stratégie (IA), gestion des actions, système de logs,
 * machine à états, ligne de commande et chronométrage du match.
 *
 * Cette classe est conçue pour être **surchargée** par chaque robot concret
 * (ex: OPOS6UL_RobotExtended) qui y injecte ses propres drivers, actionneurs,
 * stratégie et configuration via les setters (setAsserv, setActions, setSVG).
 *
 * Les méthodes virtuelles (begin, stopMotionTimerAndActionManager, freeMotion,
 * displayTS, displayObstacle) permettent à chaque robot de personnaliser
 * son comportement de démarrage, d'arrêt et d'affichage.
 *
 * La ligne de commande (préfixe /) est configurée dans configureDefaultConsoleArgs()
 * et parsée dans parseConsoleArgs(). Les sous-classes peuvent ajouter leurs
 * propres options avant l'appel à parseConsoleArgs().
 */
class Robot {
public:

	/*!
	 * \brief Retourne le \ref Logger associe a la classe \ref Robot.
	 * public car utilise dans Main.
	 */
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("Robot");
		return instance;
	}

	/*!
	 * \brief Retourne le \ref Logger RobotTelemetry4[ID] associe a la classe \ref Robot.
	 * public car utilise dans Main.
	 */

	inline const logs::Logger& telemetry()
	{
		std::ostringstream s;
		s << "RobotTelemetry4" << id_;
		const logs::Logger &telem_ = logs::LoggerFactory::logger(s.str());
		return telem_;
	}

	/*!
	 * \brief Score courant du robot pendant le match.
	 */
	int points;

	/*!
	 * \brief Indique si le robot est en mode table de test.
	 */
	bool tabletest;

	/*!
	 * \brief Indique si la fin de match (90s) doit être ignorée.
	 */
	bool skipEndOfMatch;

protected:

	utils::Chronometer chrono_;

	RobotColor myColor_;

	Arguments cArgs_;

	ConsoleManager cmanager_;

	// Create the data used to run the automate
	//Data data_;

	// Create the automate associated to the robot
	Automate automate_;

	//id of the robot
	std::string id_;

	//DATA
	bool empty_ = true;
	int useExternalEncoder_ = 1;
	int skipSetup_ = 0;
	bool end90s_ = false;
	bool lastAction_ = false;
	bool waitForInit_ = false;
	std::string strategy_ = "all"; //defaut strategy
	std::string configVRR_ = "VRR"; //defaut config VRR

	// --- Export zones simulateur (cf. ZoneJsonExporter) ---
	std::string exportZonesPath_;     ///< Chemin de sortie du table.json simulateur (vide = pas d'export)
	bool exportZonesDryRun_ = false;  ///< Si true : exit apres export, sans demarrer le match

	// --- O_State_NewInit : phase de match + config editable ---
	MatchPhase phase_ = PHASE_CONFIG;
	uint8_t    advDiameter_   = 40;           ///< Diamètre adversaire en cm, defaut Teensy-aligné.
	uint8_t    ledLuminosity_ = 10;           ///< Luminosité LED matrix 0..100, defaut Teensy-aligné.
	uint8_t    testMode_      = 0;            ///< Test materiel 0=aucun, 1..5 (transitoire).
	std::atomic<bool> testModeReq_{false};    ///< Flag "testMode a déclencher" (consumed par O_State_NewInit).
	std::atomic<bool> setPosReq_{false};      ///< Flag "passer de CONFIG a ARMED" posé par une source.
	std::atomic<bool> resetReq_{false};       ///< Flag "retour phase CONFIG" posé par une source.

	//Action => RobotElement
	Actions *actions_default_;

	ARobotPositionShared *sharedPosition_;

	TableGeometry *tableGeometry_;

	Sensors *sensors_;
private:

	//Asserv => asservissement
	Asserv *asserv_default_;

	SvgWriter *svg_;

public:
#ifdef SIMU
	int CLEF_REQUETES = 0x00012345;

	struct msgform2 {
		long mtype;
		char mtext[512];
	} msg_ipc;
#endif

	/*!
	 * \brief Constructeur de la classe.
	 */
	Robot();

	/*!
	 * \brief Destructor.
	 */
	virtual ~Robot();

	bool end90s() const
	{
		return this->end90s_;
	}
	void end90s(bool end)
	{
		this->end90s_ = end;
	}

	bool isLastAction() const
	{
		return this->lastAction_;
	}
	void lastAction(bool end)
	{
		this->lastAction_ = end;
	}

	bool isEmpty() const
	{
		return this->empty_;
	}
	void isEmpty(bool empty)
	{
		this->empty_ = empty;
	}

	std::string strategy() const
	{
		return this->strategy_;
	}
	void strategy(std::string str)
	{
		this->strategy_ = str;
	}

	std::string configVRR() const
	{
		return this->configVRR_;
	}
	void configVRR(std::string str)
	{
		this->configVRR_ = str;
	}

	const std::string& exportZonesPath() const { return exportZonesPath_; }
	bool exportZonesDryRun() const { return exportZonesDryRun_; }

	int useExternalEncoder() const
	{
		return this->useExternalEncoder_;
	}
	void useExternalEncoder(int useEncoder)
	{
		this->useExternalEncoder_ = useEncoder;
	}

	int skipSetup() const
	{
		return this->skipSetup_;
	}
	void skipSetup(int skip)
	{
		this->skipSetup_ = skip;
	}

	bool waitForInit() const
	{
		return this->waitForInit_;
	}
	void waitForInit(bool init)
	{
		this->waitForInit_ = init;
	}

	/*!
	 * \brief Retourne l'identifiant du robot.
	 */
	std::string getID()
	{
		return id_;
	}

	/*!
	 * \brief Retourne la position partagée du robot (pour communication inter-threads).
	 */
	ARobotPositionShared* sharedPosition()
	{
		return sharedPosition_;
	}

	/*!
	 * \brief Retourne un pointeur vers l'asservissement du robot.
	 */
	inline Asserv* passerv()
	{
		if (asserv_default_ == NULL) printf("ERROR asserv() NULL ! \n");
		return asserv_default_;
	}

	/*!
	 * \brief Retourne une référence vers l'asservissement du robot.
	 */
	inline Asserv& asserv()
	{
		Asserv &r_asserv = *asserv_default_;
		return r_asserv;
	}

	// Wrappers vers les transformations de couleur de l'Asserv.
	// Permettent aux modules strategie (IA, Navigator) de convertir des
	// coordonnees sans couplage direct a l'API Asserv.
	float changeMatchX(float x_mm, float width = 0.0);
	float changeMatchXMin(float x_mm, float width = 0.0);
	float changeMatchAngleRad(float rad);

	/*!
	 * \brief Retourne une référence vers le SvgWriter (logging trajectoire).
	 */
	inline SvgWriter& svgw()
	{
		SvgWriter &r_svg = *svg_;
		return r_svg;
	}

	/*!
	 * \brief Retourne une référence vers les actions du robot.
	 */
	inline Actions& actions()
	{
		Actions &r_actions = *actions_default_;
		return r_actions;
	}

	/*!
	 * \brief Définit l'asservissement du robot.
	 */
	inline void setAsserv(Asserv *asserv)
	{
		asserv_default_ = asserv;
	}

	/*!
	 * \brief Définit le SvgWriter du robot.
	 */
	inline void setSVG(SvgWriter *svg)
	{
		svg_ = svg;
	}

	/*!
	 * \brief Définit les actions du robot.
	 */
	inline void setActions(Actions *action)
	{
		actions_default_ = action;
	}

	/*!
	 * \brief Retourne la geometrie de table du robot.
	 */
	inline TableGeometry* tableGeometry()
	{
		return tableGeometry_;
	}

	/*!
	 * \brief Definit la geometrie de table du robot (propriete transferee).
	 */
	inline void setTableGeometry(TableGeometry *tg)
	{
		tableGeometry_ = tg;
	}

	/*!
	 * \brief Retourne le pointeur vers les capteurs du robot.
	 */
	inline Sensors* sensors()
	{
		return sensors_;
	}

	/*!
	 * \brief Enregistre les capteurs du robot.
	 */
	inline void setSensors(Sensors *s)
	{
		sensors_ = s;
	}

	/*!
	 * \brief Enregistre la position du robot dans le fichier SVG.
	 * \param color Couleur du tracé (0:GRIS, 1:ORANGE, 2:RED, 3:GREEN, 4:BLUE, 5:BLACK).
	 */
	void svgPrintPosition(int color = 0);

	/*!
	 * \brief Finalise et ferme le fichier SVG (ecrit les balises </g></svg>).
	 *
	 * \warning Avant d'appeler cette methode, l'appelant DOIT avoir arrete les
	 *          threads producteurs SVG (asserv CBOR, scheduler des timers),
	 *          sinon des elements <circle> seront ecrits APRES </svg> -> SVG invalide.
	 *
	 *          Sequence type :
	 *            robot.stopExtraActions();    // arrete asserv CBOR + timers
	 *            robot.svgPrintEndOfFile();   // ferme le SVG
	 *
	 *          Le destructeur ~Robot() s'en occupe automatiquement (via
	 *          stopMotionTimerAndActionManager()), donc en sortie normale du
	 *          programme rien a faire.
	 */
	void svgPrintEndOfFile();

	void operator=(Robot const&); // Don't implement

	/*!
	 * \brief Retourne le gestionnaire de console (tests fonctionnels).
	 */
	inline ConsoleManager& getConsoleManager()
	{
		ConsoleManager &r_cmanager = cmanager_;
		return r_cmanager;
	}

	/*!
	 * \brief Retourne les arguments de la ligne de commande.
	 */
	inline Arguments& getArgs()
	{
		Arguments &r_cargs = cArgs_;
		return r_cargs;
	}

	/*!
	 * \brief Cette methode retourne l'objet de manipulation du chronometer.
	 * \return Le chronometer.
	 */
	utils::Chronometer& chrono()
	{
		return chrono_;
	}

	/*!
	 * \brief Retourne la couleur du robot.
	 */
	RobotColor getMyColor() const
	{
		return myColor_;
	}

	/*!
	 * \brief Enregistre la couleur du robot (sans verification de phase).
	 *        Utilise par le parseur CLI (/b). Pour le menu runtime, preferer
	 *        setMyColorChecked() qui respecte la phase courante.
	 */
	void setMyColor(RobotColor color)
	{
		this->myColor_ = color;
	}

	// =========================================================
	// API phase de match (utilisee par O_State_NewInit + MenuController)
	// Voir robot/md/O_STATE_NEW_INIT.md section 3
	// =========================================================

	/*!
	 * \brief Retourne la phase courante du match.
	 */
	MatchPhase phase() const { return phase_; }

	/*!
	 * \brief Force la phase (appele uniquement par O_State_NewInit).
	 */
	void setPhase(MatchPhase p) { phase_ = p; }

	/*!
	 * \brief Change la couleur du robot, editable en PHASE_CONFIG seulement.
	 *        En PHASE_ARMED, le robot est deja place physiquement sur la table
	 *        selon la couleur courante, donc interdit de la changer.
	 * \return true si accepte, false si phase incompatible.
	 */
	bool setMyColorChecked(RobotColor c)
	{
		if (phase_ != PHASE_CONFIG) return false;
		myColor_ = c;
		return true;
	}

	/*!
	 * \brief Change la strategie. Editable en CONFIG + ARMED.
	 */
	bool setStrategyChecked(const std::string &s)
	{
		if (phase_ >= PHASE_MATCH) return false;
		strategy_ = s;
		return true;
	}

	/*!
	 * \brief Diametre adversaire en cm (5..250). Editable en CONFIG + ARMED.
	 */
	uint8_t advDiameter() const { return advDiameter_; }
	bool setAdvDiameter(uint8_t d)
	{
		if (phase_ >= PHASE_MATCH) return false;
		if (d < 5 || d > 250) return false;
		advDiameter_ = d;
		return true;
	}

	/*!
	 * \brief Luminosite LED matrix beacon (0..100). Editable en CONFIG + ARMED.
	 */
	uint8_t ledLuminosity() const { return ledLuminosity_; }
	bool setLedLuminosity(uint8_t l)
	{
		if (phase_ >= PHASE_MATCH) return false;
		if (l > 100) return false;
		ledLuminosity_ = l;
		return true;
	}

	/*!
	 * \brief Declenche un test meca (1..5). One-shot, reset apres consommation.
	 *        Bloque en PHASE_MATCH pour raisons de securite.
	 */
	uint8_t testMode() const { return testMode_; }
	bool triggerTestMode(uint8_t t)
	{
		if (phase_ >= PHASE_MATCH) return false;
		if (t == 0 || t > 5) return false;
		testMode_ = t;
		testModeReq_ = true;
		return true;
	}
	bool testModeRequested() const { return testModeReq_.load(); }
	void clearTestMode() { testMode_ = 0; testModeReq_ = false; }

	/*!
	 * \brief Flag setPos pose par une source (bouton BACK shield en CONFIG ou
	 *        bouton SETPOS touch). O_State_NewInit consomme pour passer en ARMED.
	 */
	void requestSetPos() { setPosReq_ = true; }
	bool setPosRequested() const { return setPosReq_.load(); }
	void clearSetPos() { setPosReq_ = false; }

	/*!
	 * \brief Flag de reset pose par une source (bouton BACK shield en ARMED ou
	 *        bouton RESET touch). O_State_NewInit revient en PHASE_CONFIG avec
	 *        freeMotion pour permettre repositionnement manuel.
	 */
	void requestReset() { resetReq_ = true; }
	bool resetRequested() const { return resetReq_.load(); }
	void clearReset() { resetReq_ = false; }

	/*!
	 * \brief Configure les options de la ligne de commande par défaut.
	 *
	 * Options disponibles :
	 * - /h : Affiche l'aide
	 * - /z : Simule les boutons (SIMU uniquement)
	 * - /k : Skip setup
	 * - /b : Couleur BLUE
	 * - /n num : Numéro du test fonctionnel
	 * - /t strategy : Nom de la stratégie de match (défaut: "all")
	 * - /i ip : IP cible télémétrie (défaut: "192.168.3.101")
	 * - /p port : Port UDP cible télémétrie (défaut: 9870)
	 */
	void configureDefaultConsoleArgs();

	/*!
	 * \brief Parse les paramètres de la ligne de commande et reconfigure
	 *        la télémétrie si /i ou /p sont fournis.
	 */
	void parseConsoleArgs(int argc, char **argv, bool stopWithErrors = true);

	/*!
	 * \brief Démarre le robot (test fonctionnel ou match).
	 */
	virtual void begin(int argc, char **argv);

	/*!
	 * \brief Arrête le timer d'asservissement et le gestionnaire d'actions.
	 */
	virtual void stopMotionTimerAndActionManager();

	/*!
	 * \brief Libère les moteurs (roue libre).
	 */
	virtual void freeMotion();

	/*!
	 * \brief Réinitialise l'affichage de l'état de trajectoire.
	 */
	virtual void resetDisplayTS();

	/*!
	 * \brief Affiche l'état de trajectoire courant.
	 */
	virtual void displayTS(TRAJ_STATE ts);

	/*!
	 * \brief Réinitialise l'affichage de détection d'obstacle.
	 */
	virtual void resetDisplayObstacle();

	/*!
	 * \brief Affiche le niveau de détection d'obstacle.
	 */
	virtual void displayObstacle(int level);

};

#endif
