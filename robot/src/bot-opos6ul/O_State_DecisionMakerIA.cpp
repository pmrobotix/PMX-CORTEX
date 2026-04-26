/*!
 * \file
 * \brief Implémentation de la classe O_State_DecisionMakerIA.
 *
 * Orchestration generique (attente init/chrono, lancement runner JSON,
 * fallback hardcode). La configuration specifique 2026 (zones de jeu,
 * callbacks d'action, actions MANIPULATION) est dans StrategyActions2026.
 */

#include "O_State_DecisionMakerIA.hpp"

#include <cstdlib>

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/FlagManager.hpp"
#include "ia/IAbyPath.hpp"
#include "ia/StrategyJsonRunner.hpp"
#include "ia/ZoneJsonExporter.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"
#include "StrategyActions2026.hpp"

O_State_DecisionMakerIA::O_State_DecisionMakerIA(Robot &robot) :
		robot_(robot)
{
}

// ============================================================================
// Boucle principale de décision
// ============================================================================

void O_State_DecisionMakerIA::execute()
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

	//wait for init!
	while (!robot.waitForInit())
	{
		utils::sleep_for_micros(1000);
	}

	logger().info() << __FUNCTION__ << " Strategy to be applied = " << robot.strategy() << logs::end;

	// Init du playground (obstacles du terrain) — INDISPENSABLE meme en mode JSON
	// (Navigator::pathTo / MOVEMENT/PATH_TO appelle playgroundFindPath).
	robot.ia().initPlayground();

	//wait for the start of the chrono !
	while (!robot.chrono().started())
	{
		utils::sleep_for_micros(10000);
	}

	logger().info() << __FUNCTION__ << " executing..." << logs::end;

	// Si /s <name> passe en CLI -> runner JSON. Sinon fallback hardcode ci-dessous.
	if (!robot.strategyJsonName().empty()) {
		logger().info() << "Using JSON strategy runner: " << robot.strategyJsonPath() << logs::end;

		// Activation de la detection adverse pour le match. setPos / pre-tirette
		// se sont deroules sans detection (SensorsThread pas demarre, lastDetection
		// reste a 0). On l'active maintenant : Asserv::waitEnd consultera
		// frontLevel / backLevel pour declencher emergency stop sur adversaire.
		robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
		robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
		robot.actions().sensors().startSensorsThread(20);

		// Enregistrement des actions MANIPULATION specifiques au reglement 2026.
		ActionRegistry actions;
		registerStrategyActions2026(actions, robot);
		logger().info() << "ActionRegistry: " << actions.size() << " actions registered" << logs::end;

		FlagManager flags;
		StrategyJsonRunner runner(&robot, &robot.ia().iAbyPath(), &actions, &flags);
		if (runner.loadFromFile(robot.strategyJsonPath())) {
			runner.run();
		} else {
			logger().error() << "JSON load failed (" << robot.strategyJsonPath()
			                 << "), fallback freeMotion" << logs::end;
		}
		robot.freeMotion();
		robot.svgPrintEndOfFile();
		logger().info() << __FUNCTION__ << " >>>>>>   svgPrintEndOfFile DONE (JSON).........." << logs::end;
		return;
	}

	logger().error() << "===== LEGACY hardcoded strategy path '" << robot.strategy()
			<< "' DEPRECATED. Use /s <PMX1|PMX2|PMX3> or pick via LCD menu. =====" << logs::end;
	robot.freeMotion();

#if 0
	// ============================================================================
	// BRANCHE LEGACY commentée : zones C++ (ia_createZone) + ia_start + hardcode
	// trajectoire d'ouverture. Remplace par le JSON runner (PMX1/2/3) au-dessus.
	// Conservé en commentaire pour reference / re-activation eventuelle si besoin
	// d'un fallback sans JSON. Voir git log pour l'historique.
	// ============================================================================

	// Mode legacy : zones C++ (ia_createZone) + ia_start. setup uniquement ici.
	// PMX1/2/3 (mode JSON) n'utilisent pas de zones C++ : les positions/actions
	// sont decrites dans strategy<name>.json (priorities + tasks).
	setupActivitiesZone2026(robot, robot.strategy());

	// Export zones simulateur (legacy uniquement : pas de zone C++ a exporter en JSON).
	if (!robot.exportZonesPath().empty()) {
		ZoneJsonExporter::exportToFile(
			robot.exportZonesPath(),
			robot.ia().iAbyPath().playground(),
			&robot.ia().iAbyPath());
		if (robot.exportZonesDryRun()) {
			logger().info() << "Dry-run: exit after export" << logs::end;
			std::exit(0);
		}
	}

	Navigator nav(&robot, &robot.ia().iAbyPath());

	TRAJ_STATE ts = TRAJ_IDLE;
	//On recule pour deposer le drapeau
	ts = nav.line(-79);

	robot.actions().ax12_GO_banderole();
	std::this_thread::sleep_for(std::chrono::seconds(1));

	robot.points += 20;
	robot.displayPoints();
	robot.asserv().setMaxSpeed(true, 40, 40);
	ts = nav.line(150);

	//On ajoute le timer de detection
	robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	robot.actions().sensors().startSensorsThread(20);

	robot.ia().iAbyPath().ia_start();        //launch IA

	robot.freeMotion();

	robot.svgPrintEndOfFile();
	logger().info() << __FUNCTION__ << " >>>>>>   svgPrintEndOfFile DONE.........." << logs::end;
#endif
}
