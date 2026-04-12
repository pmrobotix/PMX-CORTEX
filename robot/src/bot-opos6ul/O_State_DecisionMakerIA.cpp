/*!
 * \file
 * \brief Implémentation de la classe O_State_DecisionMakerIA.
 *
 * Migré depuis PMX 2025, base pour la stratégie 2026.
 * Les actions de zone (O_end_of_match_top, O_push_prise_bas) seront
 * adaptées au règlement 2026.
 */

#include "O_State_DecisionMakerIA.hpp"

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/Navigator.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

O_State_DecisionMakerIA::O_State_DecisionMakerIA(Robot &robot) :
		robot_(robot)
{
}

// ============================================================================
// Actions de zone (callbacks pour IAbyPath)
// ============================================================================

/*!
 * \brief Action de fin de match : rejoint la zone de fin en haut de table.
 *
 * Attend la fin du chrono (96s), puis avance pour marquer les points de présence.
 * \return true si l'action s'est terminée correctement, false si collision non résolue.
 */
bool O_end_of_match_top()
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	robot.logger().info() << __FUNCTION__ << logs::end;
	TRAJ_STATE ts = TRAJ_IDLE;
	ROBOTPOSITION zone;

	robot.lastAction(true);

	robot.asserv().setMaxSpeed(true, 40);

	robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

	robot.logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;

	robot.ia().iAbyPath().goToZone("zone_end_top", &zone);

	robot.displayPoints();

	Navigator nav(&robot, &robot.ia().iAbyPath());

	robot.logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;
	ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), RetryPolicy::patient());
	if (ts != TRAJ_FINISHED)
	{
		robot.logger().error() << __FUNCTION__ << " zone_end_top  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
				<< logs::end;
		robot.asserv().resetEmergencyOnTraj();
		robot.svgPrintPosition();

		return false;

	}
	robot.svgPrintPosition();

	//attente de 95sec
	while (robot.chrono().getElapsedTimeInSec() <= 96)
	{
		utils::sleep_for_secs(1);
	}

	ts = nav.line(451);
	robot.svgPrintPosition();

	robot.points += 20;
	robot.displayPoints();

	robot.svgPrintPosition();

	return true;
}

/*!
 * \brief Action de poussée de la prise en zone basse.
 *
 * Déplace le robot vers zone_prise_bas puis pousse jusqu'à la position (775, 200).
 * \return true si l'action s'est terminée, false si collision non résolue.
 */
bool O_push_prise_bas()
{
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	robot.logger().info() << __FUNCTION__ << logs::end;
	TRAJ_STATE ts = TRAJ_IDLE;
	ROBOTPOSITION zone;

	robot.asserv().setMaxSpeed(true, 40);
	robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
	robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
	robot.logger().info() << __FUNCTION__ << " start push_prise_bas x=" << zone.x << " y=" << zone.y << logs::end;
	robot.ia().iAbyPath().goToZone("zone_prise_bas", &zone);

	Navigator nav(&robot, &robot.ia().iAbyPath());
	RetryPolicy policyPrise = { 1000000, 30, 30, 0, 0, true, false };

	ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), policyPrise);
	if (ts != TRAJ_FINISHED)
	{
		robot.logger().error() << __FUNCTION__ << " zone_prise_bas  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
				<< logs::end;
		robot.asserv().resetEmergencyOnTraj();
		robot.svgPrintPosition();
		return false;
	}
	robot.svgPrintPosition();

	RetryPolicy policyPush = { 1000000, 10, 10, 0, 0, true, false };
	ts = nav.moveForwardTo(775, 200, policyPush);
	if (ts != TRAJ_FINISHED)
	{
		robot.logger().error() << __FUNCTION__ << " 775, 200  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
				<< logs::end;
		robot.asserv().resetEmergencyOnTraj();
		robot.svgPrintPosition();
		return true;
	}
	robot.svgPrintPosition();

	return true;
}

// ============================================================================
// Configuration des zones et actions
// ============================================================================

void O_State_DecisionMakerIA::IASetupActivitiesZone()
{
	logger().info() << "IASetupActivitiesZone homologation" << logs::end;
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	logger().debug() << "color = " << robot.getMyColor() << logs::end;

	robot.ia().iAbyPath().ia_createZone("zone_end_top", 150, 1550, 450, 450, 350, 1100, 90);
	robot.ia().iAbyPath().ia_createZone("zone_start", 1000, 0, 450, 450, 1300, 400, 90);
	robot.ia().iAbyPath().ia_createZone("zone_prise_bas", 550, 0, 450, 100, 775, 550, -90);

	robot.ia().iAbyPath().ia_addAction("end_of_match_top", &O_end_of_match_top);

	logger().debug() << " END IASetupActivitiesZone" << logs::end;
}

void O_State_DecisionMakerIA::IASetupActivitiesZoneTableTest()
{
	logger().error() << "IASetupActivitiesZoneTableTest !!!!!!!!!!!!!!!!!!!!!!" << logs::end;
	OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
	logger().debug() << "color = " << robot.getMyColor() << logs::end;

	robot.ia().iAbyPath().ia_createZone("zone_end_top", 150, 1550 - 420, 450, 450, 350, 1100 - 420, 90);
	robot.ia().iAbyPath().ia_createZone("zone_start", 1000, 0, 450, 450, 1300, 400, 90);
	robot.ia().iAbyPath().ia_createZone("zone_prise_bas", 550, 0, 450, 100, 775, 550, -90);

	robot.ia().iAbyPath().ia_addAction("end_of_match_top", &O_end_of_match_top);
	logger().debug() << " END IASetupActivitiesZoneTableTest !!!!!!!!!!!!!!!!!!!!!" << logs::end;
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

	// Init du playground (obstacles du terrain) pour le pathfinding
	robot.ia().initPlayground();

	if (robot.strategy() == "tabletest")
	{
		IASetupActivitiesZoneTableTest();
	}
	else if (robot.strategy() == "all")
	{
		IASetupActivitiesZone();
	}
	else
	{
		logger().error() << "NO STRATEGY " << robot.strategy() << " FOUND !!! " << logs::end;
	}

	//wait for the start of the chrono !
	while (!robot.chrono().started())
	{
		utils::sleep_for_micros(10000);
	}

	logger().info() << __FUNCTION__ << " executing..." << logs::end;

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
}
