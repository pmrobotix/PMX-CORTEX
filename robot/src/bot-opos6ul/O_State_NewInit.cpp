/*!
 * \file
 * \brief Implementation de O_State_NewInit (refactor menu multi-sources).
 *
 * Voir robot/md/O_STATE_NEW_INIT.md pour le contexte et les choix d'architecture.
 *
 * Nouveau modele a 3 phases :
 *   CONFIG -> ARMED (via setPos trigger) -> MATCH (via sequence tirette).
 *   Reset : ARMED -> CONFIG (freeMotion, couleur de nouveau editable).
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

		// setPos : robot rejoint sa position initiale
		// Reset asserv implicite via startMotionTimerAndOdo(true) dans setPos().
		setPos();
		robot.actions().sensors().writeLedLuminosity(50);

		// Debloque le DecisionMakerIA pour qu'il prepare playground/zones
		// pendant que l'operateur insere/retire la tirette.
		robot.waitForInit(true);

		robot.actions().lcd2x16().clear();
		robot.actions().lcd2x16().home();
		robot.actions().lcd2x16().print("WAIT TIRETTE...");

		// ===== PHASE ARMED : attente sequence tirette, reset possible =====
		// Couleur LOCKED. Strat/diam/LED/tests restent editables.
		// Etat interne : detecter l'edge "tirette inseree PUIS retiree".
		logger().info() << "PHASE_ARMED - insert then remove tirette" << logs::end;
		bool tiretteWasInserted = false;
		while (robot.phase() == PHASE_ARMED) {
			robot.actions().sensors().syncFull();
			ctrl.tick();
			handleTestModeRequest();

			if (robot.resetRequested()) {
				robot.clearReset();
				logger().warn() << "Reset requested -> freeMotion + restart menu" << logs::end;
				robot.asserv().freeMotion();
				goto restart_menu;
			}

			// Sequence tirette : on attend d'abord une insertion, puis le retrait.
			// pressed() == 1 : inseree, == 0 : retiree, < 0 : I2C KO -> on skip.
			int t = robot.actions().tirette().pressed();
			if (t >= 0) {
				bool isIn = (t == 1);
				if (!tiretteWasInserted && isIn) {
					tiretteWasInserted = true;
					logger().info() << "tirette inseree, waiting for release" << logs::end;
				}
				if (tiretteWasInserted && !isIn) {
					logger().info() << "tirette retiree -> PHASE_MATCH" << logs::end;
					robot.setPhase(PHASE_MATCH);
					break;
				}
			}

			if (!ctrl.anyAlive()) {
				logger().error() << "All sources lost in ARMED, EXIT" << logs::end;
				std::exit(1);
			}
			utils::sleep_for_micros(100000);
		}
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
	robot.asserv().startMotionTimerAndOdo(true);  // reset Nucleo + match ref
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
