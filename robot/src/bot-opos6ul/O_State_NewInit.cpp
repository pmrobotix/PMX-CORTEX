/*!
 * \file
 * \brief Implementation de O_State_NewInit (refactor menu multi-sources).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md pour le contexte et les choix d'architecture.
 *
 * Modele a 4 phases :
 *   CONFIG -> ARMED (via setPos trigger)
 *          -> PRIMED (tirette inseree)
 *          -> MATCH  (tirette retiree).
 *   Reset : ARMED|PRIMED -> CONFIG (freeMotion, couleur de nouveau editable).
 *
 * L'affichage "METTRE TIRETTE" / "ENLEVE TIRETTE" est gere par les sources
 * de menu (MenuShieldLCD, MenuBeaconLCDTouch) en lisant robot.phase().
 */

#include "O_State_NewInit.hpp"

#include <cstdlib>
#include <memory>

#include "action/LcdShield.hpp"
#include "action/ButtonBar.hpp"
#include "action/Sensors.hpp"
#include "action/Tirette.hpp"
#include "ia/StrategyJsonRunner.hpp"
#include "log/Logger.hpp"
#include "menu/MenuBeaconLCDTouch.hpp"
#include "menu/MenuController.hpp"
#include "menu/MenuShieldLCD.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"

#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

IAutomateState* O_State_NewInit::execute(Robot&)
{
	logger().info() << "O_State_NewInit executing..." << logs::end;

	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	robot.actions().start();

	robot.actions().lcd2x16().setBacklightOn();
	robot.actions().lcd2x16().clear();

	// Force matchState=CONFIG sur la Teensy pour faire ressortir le LCD tactile
	// du show_match_screen / show_endmatch_screen du run precedent (qui restent
	// affiches puisque la Teensy ne reboote pas avec l'OPOS6UL). Sans ce push
	// explicite, on doit attendre 1-2 ticks de la boucle menu avant que le
	// shadow comparison declenche l'ecriture, soit ~1s de fond violet apparent.
	if (robot.actions().sensors().is_connected()) {
		robot.actions().sensors().writeMatchState(static_cast<uint8_t>(PHASE_CONFIG));
		robot.actions().sensors().syncFull();
	}

	//========================================================
	// /k SKIP SETUP : params deja dans Robot via CLI
	//========================================================
	if (robot.skipSetup()) {
		logger().info() << "SKIP SETUP (/k) - params from CLI or defaults" << logs::end;
		logger().info() << "COLOR=" << (robot.getMyColor() == PMXYELLOW ? "YELLOW" : "BLUE")
				<< " STRAT=" << robot.strategy() << logs::end;

		robot.actions().lcd2x16().home();
		robot.actions().lcd2x16().print("Skip setup...");

		robot.setPhase(PHASE_ARMED);
		// Mode /k : pas de menu, l'asserv DOIT arriver avant setPos.
		// On bloque indefiniment jusqu'a la connexion (l'utilisateur peut ^C).
		waitForAsserv(nullptr);
		setPos();
		robot.actions().sensors().writeLedLuminosity(50);

		robot.waitForInit(true);
		robot.setPhase(PHASE_MATCH);
	}
	//========================================================
	// MODE MENU NORMAL (multi-sources)
	//========================================================
	else {
		MenuController ctrl(robot);

		if (robot.actions().lcd2x16().is_connected()) {
			ctrl.add(std::make_unique<MenuShieldLCD>(
					robot.actions().lcd2x16(),
					robot.actions().buttonBar()));
		} else {
			logger().warn() << "LCD shield not connected, MenuShieldLCD skipped" << logs::end;
		}

		if (robot.actions().sensors().is_connected()) {
			ctrl.add(std::make_unique<MenuBeaconLCDTouch>(robot.actions().sensors()));
		} else {
			logger().warn() << "Beacon not connected, MenuBeaconLCDTouch skipped" << logs::end;
		}

		if (!ctrl.anyAlive()) {
			logger().error() << "No menu source alive, EXIT" << logs::end;
			std::exit(1);
		}

restart_menu:
		robot.setPhase(PHASE_CONFIG);
		robot.clearSetPos();
		robot.clearReset();
		robot.clearTestMode();

		// ===== PHASE CONFIG : attente setPos =====
		// Tous les parametres editables (couleur, strat, diam, LED, tests).
		logger().info() << "PHASE_CONFIG - waiting setPos trigger" << logs::end;
		while (robot.phase() == PHASE_CONFIG) {
			robot.actions().sensors().syncFull();
			ctrl.tick();
			handleTestModeRequest();

			if (robot.setPosRequested()) {
				robot.clearSetPos();
				robot.setPhase(PHASE_ARMED);
				break;
			}
			if (!ctrl.anyAlive()) {
				logger().error() << "All sources lost in CONFIG, EXIT" << logs::end;
				std::exit(1);
			}
			utils::sleep_for_micros(100000);
		}

		// Avant setPos : la Nucleo (asserv) doit etre connectee, sinon les
		// trajectoires sont des no-op. Si pas connectee au boot, on attend
		// qu'elle arrive. L'operateur peut annuler par RESET pour repasser en
		// CONFIG (ex: changer de strategie ou debrancher / rebrancher la Nucleo).
		if (!waitForAsserv(&ctrl)) {
			logger().warn() << "WAIT NUCLEO annule par reset -> retour CONFIG" << logs::end;
			goto restart_menu;
		}

		// setPos : robot rejoint sa position initiale
		// Reset asserv implicite via startMotionTimerAndOdo(true) dans setPos().
		setPos();
		robot.actions().sensors().writeLedLuminosity(50);

		// Debloque le DecisionMakerIA pour qu'il prepare playground/zones
		// pendant que l'operateur insere/retire la tirette.
		robot.waitForInit(true);

		// ===== PHASE ARMED : attente insertion tirette, reset possible =====
		// Couleur LOCKED. Strat/diam/LED/tests restent editables.
		// Affichage "METTRE TIRETTE" gere par les sources de menu.
		logger().info() << "PHASE_ARMED - waiting tirette insertion" << logs::end;
		while (robot.phase() == PHASE_ARMED) {
			robot.actions().sensors().syncFull();
			ctrl.tick();
			handleTestModeRequest();

			if (robot.resetRequested()) {
				robot.clearReset();
				logger().warn() << "Reset requested in ARMED -> freeMotion + restart menu" << logs::end;
				robot.asserv().freeMotion();
				goto restart_menu;
			}

			// pressed() == 1 : inseree, == 0 : retiree, < 0 : I2C KO -> on skip.
			int t = robot.actions().tirette().pressed();
			if (t == 1) {
				logger().info() << "tirette inseree -> PHASE_PRIMED" << logs::end;
				robot.setPhase(PHASE_PRIMED);
				break;
			}

			if (!ctrl.anyAlive()) {
				logger().error() << "All sources lost in ARMED, EXIT" << logs::end;
				std::exit(1);
			}
			utils::sleep_for_micros(100000);
		}

		// ===== PHASE PRIMED : attente retrait tirette, reset possible =====
		// Meme regles d'edition que ARMED (couleur LOCKED).
		// Affichage "ENLEVE TIRETTE" gere par les sources de menu.
		logger().info() << "PHASE_PRIMED - waiting tirette release" << logs::end;
		while (robot.phase() == PHASE_PRIMED) {
			robot.actions().sensors().syncFull();
			ctrl.tick();
			handleTestModeRequest();

			if (robot.resetRequested()) {
				robot.clearReset();
				logger().warn() << "Reset requested in PRIMED -> freeMotion + restart menu" << logs::end;
				robot.asserv().freeMotion();
				goto restart_menu;
			}

			int t = robot.actions().tirette().pressed();
			if (t == 0) {
				logger().info() << "tirette retiree -> PHASE_MATCH" << logs::end;
				robot.setPhase(PHASE_MATCH);
				break;
			}

			if (!ctrl.anyAlive()) {
				logger().error() << "All sources lost in PRIMED, EXIT" << logs::end;
				std::exit(1);
			}
			utils::sleep_for_micros(100000);
		}

		// Flush final : pousse PHASE_MATCH vers toutes les sources avant de
		// quitter le menu (sinon le LCD tactile balise reste sur "ENLEVE TIR"
		// car ctrl.tick() n'est plus appele apres ce bloc).
		ctrl.tick();
		robot.actions().sensors().syncFull();
	}

	//========================================================
	// Depart match (commun /k et menu) — inchange par rapport a O_State_Init
	//========================================================
	robot.actions().ledBar().stop(true);
	robot.actions().ledBar().resetAll();
	robot.actions().lcd2x16().clear();
	robot.actions().lcd2x16().setBacklightOn();
	robot.actions().lcd2x16().setCursor(0, 0);
	robot.actions().lcd2x16().home();
	robot.actions().lcd2x16().print("GO...");

	robot.displayPoints();

	logger().info() << "O_State_NewInit executed" << logs::end;
	return this->getState("WaitEndOfMatch");
}

bool O_State_NewInit::waitForAsserv(MenuController* ctrl)
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	// Delai de stabilisation post-tryReconnect : la Nucleo qui vient de booter
	// envoie deja ses frames de position (thread SerialCbor::positionOutput
	// demarre tot), mais son CommandManager peut ne pas encore etre pret a
	// accepter de nouvelles commandes (init encoders/PID/moteurs ~500ms-1s).
	// Sans ce delai, setPositionAndColor envoye trop tot est ignore silencieusement
	// par la Nucleo -> received_cmd reste a 0 indefiniment -> waitEndOfTraj
	// timeout 10s sur les commandes suivantes.
	// TODO: remplacer par un handshake explicite (emergency_stop_reset + ACK)
	// pour avoir confirmation que la Nucleo est prete avant de continuer.
	constexpr int NUCLEO_STABILIZATION_US = 1000000;  // 1s

	if (robot.asserv().tryReconnect()) {
		logger().info() << "Asserv deja connectee, stabilisation Nucleo (1s)..." << logs::end;
		utils::sleep_for_micros(NUCLEO_STABILIZATION_US);
		return true;
	}

	logger().warn() << "Asserv non connectee, attente Nucleo..." << logs::end;
	robot.actions().lcd2x16().clear();
	robot.actions().lcd2x16().home();
	robot.actions().lcd2x16().print("WAIT NUCLEO...");

	int retryCount = 0;
	while (!robot.asserv().tryReconnect()) {
		if (ctrl) {
			// pollInputsOnly (sans refreshDisplay) : on detecte un reset
			// eventuel sans que les sources ecrasent "WAIT NUCLEO..." par
			// leur layout ARMED standard ("METTRE TIRETTE !").
			robot.actions().sensors().syncFull();
			ctrl->pollInputsOnly();
			if (robot.resetRequested()) {
				robot.clearReset();
				logger().warn() << "Reset pendant WAIT NUCLEO -> annulation" << logs::end;
				return false;
			}
		}
		// 500ms entre 2 tentatives. tryReconnect inclut deja un readBytes(500ms)
		// si openDevice reussit, donc en pratique le polling est plus lent quand
		// la Nucleo est presente mais lente a repondre.
		utils::sleep_for_micros(500000);
		if (++retryCount % 10 == 0) {
			logger().info() << "WAIT NUCLEO... retry #" << retryCount << logs::end;
		}
	}

	logger().info() << "Nucleo connectee !" << logs::end;
	robot.actions().lcd2x16().clear();
	logger().info() << "Stabilisation Nucleo (1s)..." << logs::end;
	utils::sleep_for_micros(NUCLEO_STABILIZATION_US);

	// Securite : si on est passe par "WAIT NUCLEO..." (Nucleo allumee tardivement),
	// le robot peut avoir subi un glitch d'allumage (sursaut moteur) ou avoir ete
	// deplace en raison de l'ARU lache d'un coup. Faire setPos directement avec
	// la pose initiale (230, 130, 90°) supposerait que le robot y est encore,
	// ce qui n'est plus garanti. On force l'operateur a repositionner le robot
	// manuellement puis a re-cliquer SETPOS. freeMotion libere les moteurs.
	// Mode /k (ctrl == nullptr) : pas de re-positionnement possible, on continue.
	if (ctrl) {
		logger().warn() << "Reconnect tardif Nucleo -> retour CONFIG, "
				<< "repositionner robot puis SETPOS a nouveau" << logs::end;
		robot.actions().lcd2x16().clear();
		robot.actions().lcd2x16().home();
		robot.actions().lcd2x16().print("REPOSITIONNER");
		robot.actions().lcd2x16().setCursor(0, 1);
		robot.actions().lcd2x16().print("PUIS SETPOS");
		robot.asserv().freeMotion();
		// Le caller (boucle CONFIG -> ARMED) fait goto restart_menu sur false.
		// Note : on laisse le LCD afficher le message ; le menu va le re-ecrire
		// au prochain tick CONFIG (couleur, strat, diam) mais le freeMotion +
		// retour en CONFIG suffit a indiquer a l'operateur qu'il faut recommencer.
		return false;
	}
	return true;
}

void O_State_NewInit::handleTestModeRequest()
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	if (!robot.testModeRequested()) return;

	uint8_t t = robot.testMode();
	robot.clearTestMode();
	logger().info() << "testMode request t=" << (int)t << logs::end;

	// Dispatch vers routines numerotees (a coder progressivement).
	// Convention : chaque routine doit "self-clean" la meca en fin d'execution
	// pour permettre re-lancement repete.
	switch (t) {
		case 1:
			robot.actions().ax12_init();
			break;
		case 2:
			// TODO: test capteurs ToF / beacon comm
			break;
		case 3:
			// TODO: test aspiration / turbine
			break;
		case 4:
			// TODO: test LED bar
			break;
		case 5:
			// TODO: test servos objets
			break;
		default:
			logger().warn() << "Unknown testMode " << (int)t << logs::end;
			break;
	}
}

// setPos : copie fonctionnelle de O_State_Init::setPos(), avec reset asserv
// implicite via startMotionTimerAndOdo(true). Appele a chaque transition
// CONFIG -> ARMED (y compris apres un reset).
void O_State_NewInit::setPos()
{
	logger().info() << "O_State_NewInit::setPos() executing" << logs::end;
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	const bool useJsonInit = !robot.strategyJsonName().empty();

	robot.actions().lcd2x16().clear();
	robot.actions().lcd2x16().print("SET POSITION...");

	robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

	// Pose initiale : depuis init<name>.json si /s passe, sinon hardcode historique.
	// setPositionAndColor AVANT startMotionTimerAndOdo (reset Nucleo + match ref).
	const float ix  = useJsonInit ? robot.initPoseX()        : 300.0f;
	const float iy  = useJsonInit ? robot.initPoseY()        : 130.0f;
	const float ith = useJsonInit ? robot.initPoseThetaDeg() : 90.0f;
	logger().info() << "setPos pose source=" << (useJsonInit ? "JSON" : "hardcode")
			<< " x=" << ix << " y=" << iy << " theta=" << ith << "deg" << logs::end;
	robot.asserv().setPositionAndColor(ix, iy, ith, robot.isMatchColor());
	robot.asserv().startMotionTimerAndOdo(true);  // reset Nucleo + match ref + 200ms stabilisation Nucleo

	ROBOTPOSITION p = robot.sharedPosition()->getRobotPosition();
	logger().info() << "setPos() svgPrintPosition x=" << p.x << " y=" << p.y
			<< " a=" << radToDeg(p.theta) << logs::end;
	robot.svgPrintPosition();

	robot.actions().lcd2x16().clear();

	robot.asserv().assistedHandling();

	robot.actions().ax12_init();

	robot.asserv().setMaxSpeed(true, 50);

	if (useJsonInit) {
		// Mode /s : pre-tirette pilote par setpos_tasks du JSON init.
		// Tableau vide = aucun mouvement (intentionnel).
		if (!robot.setposTasks().empty()) {
			StrategyJsonRunner runner(&robot, &robot.ia().iAbyPath());
			if (!runner.runTasks(robot.setposTasks(), "SETPOS")) {
				robot.logger().error() << "setPos : SETPOS tasks abort" << logs::end;
				robot.asserv().resetEmergencyOnTraj();
			}
		} else {
			logger().info() << "setPos : setpos_tasks vide (pas de mouvement pre-tirette)" << logs::end;
		}
	} else {
		// Mode legacy (sans /s, ni PMX* via menu) : compat retro hardcode "avance 80mm".
		logger().warn() << "===== LEGACY hardcoded setPos (no JSON strategy) DEPRECATED. "
				<< "Use /s <PMX1|PMX2|PMX3> or pick via LCD menu. =====" << logs::end;
		Navigator nav(&robot);
		TRAJ_STATE ts = nav.line(80);
		if (ts != TRAJ_FINISHED) {
			robot.logger().error() << "setPos : ===== PB COLLISION FINALE ts=" << ts << logs::end;
			robot.asserv().resetEmergencyOnTraj();
		}
	}

	robot.svgPrintPosition();
	robot.actions().lcd2x16().println("SET POSITION : OK");

	robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
}
