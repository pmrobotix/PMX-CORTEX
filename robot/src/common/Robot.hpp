#ifndef COMMON_ROBOT_HPP_
#define COMMON_ROBOT_HPP_

#include <stdio.h>
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

/*!
 * \brief Couleur de l'équipe du robot sur la table.
 */
enum RobotColor {
	PMXNOCOLOR, PMXYELLOW, PMXBLUE
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

	//Action => RobotElement
	Actions *actions_default_;

	ARobotPositionShared *sharedPosition_;
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

	/*!
	 * \brief Fonctions de motion intelligente en plusieurs essais.
	 */
	TRAJ_STATE whileDoLine(float distMM, bool rotate_ignoring_opponent = true, int wait_tempo_us = 2000000,
	    		int nb_near_obstacle = 2, int nb_collision = 2, int reculOnObstacleMm = 0, int reculOnCollisionMm = 0,
	    		bool ignore_collision = 0);

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
	 * \brief Enregistre la position du robot dans le fichier SVG.
	 * \param color Couleur du tracé (0:GRIS, 1:ORANGE, 2:RED, 3:GREEN, 4:BLUE, 5:BLACK).
	 */
	void svgPrintPosition(int color = 0);

	/*!
	 * \brief Finalise et ferme le fichier SVG.
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
	 * \brief Enregistre la couleur du robot.
	 */
	void setMyColor(RobotColor color)
	{
		this->myColor_ = color;
	}

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
