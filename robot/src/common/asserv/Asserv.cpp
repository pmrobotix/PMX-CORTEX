#include "Asserv.hpp"

#include <stdlib.h>
#include <thread>

#include "asserv.esial/AsservEsialR.hpp"
#include "log/Logger.hpp"
#include "thread/Thread.hpp"
#include "../Robot.hpp"
#include "action/Sensors.hpp"
#include "geometry/DetectionEvent.hpp"

// Constructeur : instancie le driver d'asserv (ext ou interne) et initialise les paramètres par défaut.
// Le type d'asserv par défaut est ASSERV_INT_ESIALR (asservissement interne ESIAL).
Asserv::Asserv(std::string botId, Robot *robot)
{
	// Création du driver de communication (série/simu) selon le botId
	asservdriver_ = AAsservDriver::create(botId, robot->sharedPosition());
	probot_ = robot;

	useAsservType_ = ASSERV_INT_ESIALR; // par défaut : asserv interne ESIAL
	emergencyStop_ = false;

	// Instanciation de l'asserv interne si sélectionnée
	if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_ = new AsservEsialR(robot);
	else
		pAsservEsialR_ = NULL;


	matchColorPosition_ = false; // couleur primaire par défaut

	adv_pos_centre_ = { -100.0, -100.0, 0, 0 }; // position adversaire inconnue

	// Dimensions de la table : 3000mm en horizontal (Coupe de France standard)
	x_ground_table_ = 3000;

	lowSpeedvalue_ = 30;          // % vitesse lente (surchargé par chaque robot)
	maxSpeedDistValue_ = 50;      // deprecated : ancien % PWM cap pour reduction obstacle
	obstacleSpeedPercent_ = 40;   // % acc/dec quand obstacle detecte (configurable)
	userAccDecPercent_ = 100;     // % acc/dec courant cote utilisateur (default 100)
	userMaxSpeedActive_ = false;  // par defaut pas de cap PWM (NORMAL_SPEED_ACC)
	userMaxSpeedPercent_ = 100;

}
Asserv::~Asserv()
{
	delete asservdriver_;
	delete pAsservEsialR_;
}
/*
 //TODO utiliser convertPositionBeaconToRepereTable dans ARobotPositionShared ?
 ROBOTPOSITION Asserv::convertPositionToRepereTable(float d_mm, float x_mm, float y_mm, float theta_deg, float *x_botpos,
 float *y_botpos)
 {
 ROBOTPOSITION p = pos_getPosition();
 //coordonnées de l'objet detecté sur la table// M_P/2
 //    *x_botpos = p.x + (d_mm * cos(p.theta - M_PI_2 + (theta_deg * M_PI / 180.0f)));
 //    *y_botpos = p.y + (d_mm * sin(p.theta - M_PI_2 + (theta_deg * M_PI / 180.0f)));
 float a = (p.theta - M_PI_2 + (theta_deg * M_PI / 180.0f));
 std::fmod(a, 2 * M_PI);
 //    if (a < -M_PI)
 //        a += M_PI;
 //    if (a > M_PI)
 //        a -= M_PI;

 //ADV coord
 float fx_botpos= p.x + (d_mm * cos(a));
 float fy_botpos = p.y + (d_mm * sin(a));

 *x_botpos  = fx_botpos;
 *y_botpos = fy_botpos;
 logger().debug() << "DEBUG --xy_botpos= " << *x_botpos << " " << *y_botpos
 << " pos: " << p.x << " " << p.y << " p_deg:" << p.theta  * 180.0f / M_PI<< " --balise: " << d_mm << " " << x_mm << " "
 << y_mm << " t_deg:" << theta_deg << logs::end;


 return p;
 }
 */

// Appelé en fin de trajectoire. Délègue au driver actif.
void Asserv::endWhatTodo()
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{

		pAsservEsialR_->endWhatTodo();
	} else if (useAsservType_ == ASSERV_EXT)
	{

		asservdriver_->endWhatTodo();
	}
}

void Asserv::startMotionTimerAndOdo(bool assistedHandlingEnabled)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{

		//TO BE surcharged because of the specific config file per robot
		logger().error() << "TODO startMotionTimerAndOdo  ASSERV_INT_ESIALR  TO BE surcharged !!!" << logs::end;

//        pAsservEsialR_->motion_ActivateManager(true); //on active le thread
//        if (assistedHandlingEnabled) pAsservEsialR_->motion_AssistedHandling();
//        else pAsservEsialR_->motion_FreeMotion();
	} else if (useAsservType_ == ASSERV_EXT)
	{
		// Note : pour les robots qui surchargent (ex: OPOS6UL_AsservExtended),
		// cette branche n'est PAS executee — seule la version overridee tourne.
		asservdriver_->motion_ActivateManager(true); //on active la carte d'asserv externe et le thread de position
		if (assistedHandlingEnabled)
			asservdriver_->motion_AssistedHandling();
		else
			asservdriver_->motion_FreeMotion();
	}
}

// Arrête le thread d'asservissement et l'odométrie.
// EXT : désactive le manager de mouvement.
// INT_ESIALR : stoppe l'asserv interne.
void Asserv::stopMotionTimerAndOdo()
{
	if (useAsservType_ == ASSERV_EXT)
	{
		asservdriver_->motion_ActivateManager(false);
	} else if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->stopAsserv();
	}
}

void Asserv::setLowSpeedForward(bool enable, int percent)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->motion_setLowSpeedForward(enable, percent);

	} else if (useAsservType_ == ASSERV_EXT) asservdriver_->motion_setLowSpeedForward(enable, percent);
}

void Asserv::setLowSpeedBackward(bool enable, int percent)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->motion_setLowSpeedBackward(enable, percent);
	} else if (useAsservType_ == ASSERV_EXT) asservdriver_->motion_setLowSpeedBackward(enable, percent);
}

void Asserv::setMaxSpeed(bool enable, int speed_dist_percent, int speed_angle_percent)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		//TODO setMaxSpeed ASSERV_INT_ESIALR

	} else if (useAsservType_ == ASSERV_EXT)
	{
		if (enable)
		{
			if (speed_dist_percent > 100) speed_dist_percent = 100;
			if (speed_angle_percent > 100) speed_angle_percent = 100;

			logger().info() << "setMaxSpeed [PWM cap] dist=" << speed_dist_percent
					<< "% angle=" << speed_angle_percent
					<< "% (cmd CBOR 17, note: angle ignore cote driver)" << logs::end;
			asservdriver_->motion_setMaxSpeed(true, speed_dist_percent, speed_angle_percent);
			userMaxSpeedActive_ = true;                 // memorise pour restore
			userMaxSpeedPercent_ = speed_dist_percent;

		} else
		{
			logger().info() << "setMaxSpeed [PWM cap] DISABLED -> NORMAL_SPEED_ACC (cmd CBOR 15)" << logs::end;
			asservdriver_->motion_setMaxSpeed(false);
			userMaxSpeedActive_ = false;                 // memorise pour restore
		}
	}
}

void Asserv::setAccDecPercent(int percent)
{
	if (useAsservType_ == ASSERV_EXT)
	{
		if (percent < 1)   percent = 1;
		if (percent > 100) percent = 100;
		logger().info() << "setAccDecPercent [scale acc/dec amont PID] " << percent
				<< "% (cmd CBOR 18, dist + angle)" << logs::end;
		asservdriver_->motion_setAccDecPercent(percent);
		userAccDecPercent_ = percent;     // memorise pour restore apres obstacle
	}
	// ASSERV_INT_ESIALR : pas de support actuel, no-op
}

void Asserv::setObstacleSpeedPercent(int percent)
{
	if (percent < 1)   percent = 1;
	if (percent > 100) percent = 100;
	obstacleSpeedPercent_ = percent;
	logger().info() << "setObstacleSpeedPercent: " << percent
			<< "% (acc/dec applique sur detection obstacle)" << logs::end;
}

int Asserv::getObstacleSpeedPercent() const
{
	return obstacleSpeedPercent_;
}

void Asserv::setSpeed(int percent)
{
	// Implementation : choix a ajuster selon les tests.
	// Default : applique les 2 mecanismes (cap PWM + scale acc/dec).
	// Pour tester un seul, commenter l'autre.
	setMaxSpeed(true, percent, percent);
	//setAccDecPercent(percent);
}

void Asserv::applySpeedSnapshotDirect(bool maxSpeedActive, int maxSpeedPct, int accDecPct)
{
	if (useAsservType_ != ASSERV_EXT) return;
	asservdriver_->motion_setAccDecPercent(accDecPct);
	asservdriver_->motion_setMaxSpeed(maxSpeedActive, maxSpeedPct, maxSpeedPct);
}

void Asserv::disablePID() //deprecated and ActivateQuanramp to be defined
{

	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->motion_ActivateQuadRamp(false);
	} else if (useAsservType_ == ASSERV_EXT)
	{
		freeMotion();
	}
}

void Asserv::freeMotion()
{
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_FreeMotion();
	else if (useAsservType_ == ASSERV_INT_ESIALR) pAsservEsialR_->motion_FreeMotion();
}

void Asserv::assistedHandling()
{
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_AssistedHandling();
	else if (useAsservType_ == ASSERV_INT_ESIALR) pAsservEsialR_->motion_AssistedHandling();
}

// Définit la position initiale et applique la symétrie couleur de match.
// matchColor=0 (primaire) : coordonnées telles quelles (bas-gauche du terrain).
// matchColor=1 (secondaire) : X est miroir (3000-X), angle est miroir (PI-angle).
void Asserv::setPositionAndColor(float x_mm, float y_mm, float thetaInDegrees_, bool matchColor = 0)
{
	// Reset etat residuel Nucleo (queue motion + sync nextCmdId_) une seule fois
	// au tout 1er setPositionAndColor du process. Doit etre AVANT setPositionReal :
	// sans ca, le set_position serait noye dans la rafale de motions residuelles
	// que la Nucleo execute encore depuis sa queue precedente.
	if (useAsservType_ == ASSERV_EXT && !nucleoResetDone_) {
		asservdriver_->resetNucleoState();
		nucleoResetDone_ = true;
	}

	setMatchColorPosition(matchColor);

	x_mm = changeMatchX(x_mm);
	float thetaInRad = changeMatchAngleRad(degToRad(thetaInDegrees_));

	logger().debug() << "matchcolor [BLUE=0 YELLOW=1]=" << matchColor << " thetaInDegrees=" << thetaInDegrees_
			<< " getRelativeAngle=" << radToDeg(thetaInRad) << " x_mm=" << x_mm << " y_mm=" << y_mm << logs::end;

//    if (useAsservType_ == ASSERV_EXT)
//        asservdriver_->odo_SetPosition(x_mm, y_mm, thetaInRad);
//    else if (useAsservType_ == ASSERV_INT_ESIALR)
//        pAsservEsialR_->odo_SetPosition(x_mm, y_mm, thetaInRad);
	setPositionReal(x_mm, y_mm, thetaInRad);
}

// =============================================================================
// setPositionReal — push local fort + 100ms d'attente (option A)
// =============================================================================
//
// La cmd CBOR set_position est appliquee SYNCHRONEMENT cote Nucleo
// (Odometry::setPosition simple assignment a m_X/m_Y/m_theta_rad). Pas de
// handshake cmd_id necessaire.
//
// Probleme reel : la frame CBOR position en transit AVANT que la Nucleo ne
// recoive le set_position contient encore l'ancienne pose. Si on validait
// sur cette frame "fraiche" (counter ++), on accepterait un etat stale.
//
// Approche option A :
//   1. Envoie odo_SetPosition (la Nucleo applique son odometrie immediatement).
//   2. Push local pTarget dans sharedPosition (la pose lue par les caller
//      pos_getPosition() pointe pile sur la cible des le retour).
//   3. Sleep 100 ms : laisse passer la frame Nucleo en transit (10 Hz =
//      1 frame max dans 100 ms). Les frames suivantes contiennent la pose
//      post-set.
//   4. Re-push pTarget : pendant le sleep, la frame stale a peut-etre ecrase
//      sharedPosition. On reaffirme la cible exacte avant le retour. Les
//      frames Nucleo suivantes sont post-set (proches de pTarget a +/-1mm
//      de bruit odo).
//
void Asserv::setPositionReal(float x_mm, float y_mm, float thetaInRad)
{
	if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->odo_SetPosition(x_mm, y_mm, thetaInRad);
		ROBOTPOSITION p = {x_mm, y_mm, thetaInRad, 0, 0, 0};
		probot_->sharedPosition()->setRobotPosition(p);
		return;
	}
	if (useAsservType_ != ASSERV_EXT) return;

	const ROBOTPOSITION pTarget = {x_mm, y_mm, thetaInRad, 0, 0, 0};

	asservdriver_->odo_SetPosition(x_mm, y_mm, thetaInRad);
	probot_->sharedPosition()->setRobotPosition(pTarget);

	utils::sleep_for_micros(100000); // 100 ms : purge frame Nucleo en transit

	probot_->sharedPosition()->setRobotPosition(pTarget); // re-affirme la cible exacte

	logger().debug() << "setPositionReal: target ("
			<< x_mm << ", " << y_mm << ", " << thetaInRad
			<< ") pushed + 100ms wait" << logs::end;
}

ROBOTPOSITION Asserv::pos_getAdvPosition()
{
	return adv_pos_centre_;
}

ROBOTPOSITION Asserv::pos_getPosition()
{
	// Source de verite unique : sharedPosition. Elle est alimentee par :
	//  - setPositionReal() qui push immediatement la valeur locale (sans attendre CBOR)
	//  - le thread du driver (AsservCborDriver/AsservEsialR) a chaque trame recue
	// Lire ici directement au lieu de asservdriver_->odo_GetPosition() evite un
	// decalage au demarrage : sinon, juste apres setPositionAndColor, le driver
	// retournerait encore (0,0,0) tant que la Nucleo n'a pas renvoye sa 1ere trame.
	return probot_->sharedPosition()->getRobotPosition();
}
float Asserv::pos_getX_mm()
{
	ROBOTPOSITION p = pos_getPosition();
	return p.x;
}
float Asserv::pos_getY_mm()
{
	ROBOTPOSITION p = pos_getPosition();
	return p.y;
}
// angle in radian
float Asserv::pos_getTheta()
{
	ROBOTPOSITION p = pos_getPosition();
	return p.theta;
}

// angle in degrees
float Asserv::pos_getThetaInDegree()
{
	return (pos_getTheta() * 180.0f) / M_PI;
}

void Asserv::setEmergencyStop()
{
	logger().debug() << "Asserv::setEmergencyStop() !!!!!!!!!!!" << logs::end;
	if (emergencyStop_ == true)
	{
		logger().debug() << "Asserv::setEmergencyStop() emergencyStop_ ALREADY TRUE!" << logs::end;
		return;
	} else
	{
		emergencyStop_ = true;

		if (useAsservType_ == ASSERV_EXT)
			asservdriver_->emergencyStop();
		else if (useAsservType_ == ASSERV_INT_ESIALR) pAsservEsialR_->emergencyStop();
	}

}

void Asserv::resetEmergencyOnTraj(std::string message)
{
	if (emergencyStop_ == false)
	{
		logger().debug() << "Asserv::resetEmergencyOnTraj() emergencyStop_ IS NOT TRUE!" << logs::end;
		return;
	}
	logger().debug() << "=====   resetEmergencyOnTraj message = " << message << logs::end;
	emergencyStop_ = false;
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->resetEmergencyStop();
	else if (useAsservType_ == ASSERV_INT_ESIALR) pAsservEsialR_->resetEmergencyStop();
}

// =============================================================================
// sendCborMotionWithRetry — Phase A handshake + warn (driver CBOR)
// =============================================================================
//
// Wrapper autour des motion_* du driver pour fiabiliser la communication :
//
//   1. Envoi de la commande (sendFn → motion_Line/Rotate/GoTo/...).
//   2. Phase A : attendre l'ACK queue (lastReceivedCmdId >= targetCmdId)
//      avec timeout long (1500ms ~ couvre la latence reelle Nucleo observee).
//   3. Phase B = waitEndOfTrajWithDetection (existant), peu importe le
//      resultat de Phase A. Le but de Phase A est purement diagnostique :
//      logger un WARN si la cmd met trop de temps a etre acquittee.
//
// Pas de retry automatique : un retry creerait une 2eme cmd avec un nouveau
// cmd_id, que la Nucleo empilerait dans sa queue -> double execution
// catastrophique (cf bug observe : cascading retries qui font reculer +
// avancer le robot). Si la cmd est vraiment perdue, c'est waitEndOfTraj
// qui timeout en 10s ; l'appelant peut decider de retry au niveau strategie.
//
// La protection contre la perte de cmd reste le sleep 200ms inter-task
// dans StrategyJsonRunner (qui evite la fenetre overflow Rx Nucleo).
//
TRAJ_STATE Asserv::sendCborMotionWithRetry(MovementType type, std::function<void()> sendFn)
{
	constexpr int ACK_TIMEOUT_MS = 1500;   // couvre latence reelle Nucleo (~400-800ms)

	sendFn();
	const int targetCmdId = asservdriver_->lastSentCmdId();

	if (!asservdriver_->waitForCmdAck(targetCmdId, ACK_TIMEOUT_MS)) {
		logger().warn() << "sendCborMotionWithRetry: cmd " << targetCmdId
				<< " no ACK in " << ACK_TIMEOUT_MS << "ms (received="
				<< asservdriver_->lastReceivedCmdId()
				<< ") - waitEndOfTraj continuera mais risque timeout 10s" << logs::end;
	}

	return waitEndOfTrajWithDetection(type);
}

// =============================================================================
// waitEndOfTrajWithDetection — boucle centrale de décision
// =============================================================================
//
// Modele cmd_id ack (cf EsialRobotik/Ia-Python::wait_for_asserv) :
//   La commande motion_* qui a precede cet appel a incremente nextCmdId_ cote
//   driver. On capture ce target_id et on attend que la Nucleo nous renvoie
//   un cmd_id >= target_id ET reporte status==IDLE. C'est l'ACK explicite.
//   Plus de Phase 1 "wait RUNNING" : un IDLE stale d'avant la commande ne
//   peut plus etre confondu avec "termine" parce que lastReceivedCmdId() est
//   alors strictement < target_id.
//
//   Le wrapper SIMU n'a pas de cmd_id : ses defaults retournent 0/0, la
//   condition "received(0) >= target(0)" est triviale et c'est le status==IDLE
//   qui decide (cf force-status=1 a l'enqueue dans AsservDriverSimu).
//
// La detection ToF/beacon (frontLevel/backLevel) reste integree dans la meme
// boucle pour declencher emergency stop sur obstacle proche.
//
// Status driver : 0=IDLE, 1=RUNNING, 2=EMERGENCY, 3=BLOCKED.

TRAJ_STATE Asserv::waitEndOfTrajWithDetection(MovementType type)
{
	// Capture du cmd_id cible AU DEBUT (juste apres l'envoi de la commande
	// par l'appelant motion_*). Tout ACK >= cette valeur signifie que la
	// commande a ete consommee par la Nucleo.
	const int targetCmdId = asservdriver_->lastSentCmdId();

	// Etat local pour la regulation de vitesse selon detection : on ne
	// renvoie une cmd CBOR que si l'etat CHANGE, pas a chaque tick 10ms.
	// Sinon -> spam serie qui sature la Nucleo et ralentit la trajectoire.
	bool speedReducedActive = false;   // false = user value, true = obstacle reduce

	// Snapshot des 2 reglages vitesse user a l'entree -> restaures EXACTEMENT
	// quand obstacle clear ou fin de traj (option B : save/restore complete).
	const int  savedUserAccDecPercent  = userAccDecPercent_;
	const bool savedUserMaxSpeedActive = userMaxSpeedActive_;
	const int  savedUserMaxSpeedPct    = userMaxSpeedPercent_;

	int timeout = 0;
	while (true)
	{
		ROBOTPOSITION p = pos_getPosition();
		const int status = p.asservStatus;
		const int receivedCmdId = asservdriver_->lastReceivedCmdId();

		// --- Fini normalement : ACK cmd_id + status IDLE ---
		// Pour SIMU (defaults 0/0), receivedCmdId(0) >= targetCmdId(0) est
		// trivialement vrai -> seul le status==0 decide, comme avant.
		if (receivedCmdId >= targetCmdId && status == 0)
		{
			// Restaurer les 2 reglages user UNIQUEMENT si on l'avait reduit a
			// cause d'un obstacle (le snapshot reste la verite).
			if (speedReducedActive) {
				applySpeedSnapshotDirect(savedUserMaxSpeedActive,
						savedUserMaxSpeedPct, savedUserAccDecPercent);
				speedReducedActive = false;
			}
			return TRAJ_FINISHED;
		}

		// --- Blocage moteur (collision physique) ---
		if (status == 3) // BLOCKED
		{
			return TRAJ_COLLISION;
		}

		// --- Emergency stop externe (déclenché par un autre appel) ---
		if (status == 2) // EMERGENCY
		{
			return TRAJ_INTERRUPTED;
		}

		// --- Consultation du DetectionEvent (publié par SensorsThread) ---
		Sensors* sensors = probot_->sensors();
		if (sensors != nullptr)
		{
			const DetectionEvent& det = sensors->lastDetection();

			if (type == FORWARD)
			{
				if (det.frontLevel == 4)
				{
					setEmergencyStop();
					sensors->setStopDetection(det);
					logger().debug() << "waitEndOfTrajWithDetection FORWARD: STOP level 4"
							<< " adv=(" << det.x_adv_mm << "," << det.y_adv_mm << ")" << logs::end;
					return TRAJ_NEAR_OBSTACLE;
				}
				const bool wantReduced = (det.frontLevel >= 3);
				if (wantReduced != speedReducedActive)
				{
					if (wantReduced) {
						// Reduction : scale acc/dec a obstacleSpeedPercent_ (cap PWM
						// user inchange). Appel direct driver -> ne touche pas user*.
						asservdriver_->motion_setAccDecPercent(obstacleSpeedPercent_);
						logger().info() << "waitEndOfTraj FORWARD: obstacle DETECTED -> setAccDecPercent "
								<< obstacleSpeedPercent_ << "%" << logs::end;
					} else {
						// Clear : restore EXACTEMENT les 2 reglages user.
						applySpeedSnapshotDirect(savedUserMaxSpeedActive,
								savedUserMaxSpeedPct, savedUserAccDecPercent);
						logger().info() << "waitEndOfTraj FORWARD: obstacle CLEAR -> restore user (acc="
								<< savedUserAccDecPercent << "% pwmCap="
								<< (savedUserMaxSpeedActive ? savedUserMaxSpeedPct : 0)
								<< "%)" << logs::end;
					}
					speedReducedActive = wantReduced;
				}
			}
			else if (type == BACKWARD)
			{
				if (det.backLevel == -4)
				{
					setEmergencyStop();
					sensors->setStopDetection(det);
					logger().debug() << "waitEndOfTrajWithDetection BACKWARD: STOP level -4"
							<< " adv=(" << det.x_adv_mm << "," << det.y_adv_mm << ")" << logs::end;
					return TRAJ_NEAR_OBSTACLE;
				}
				const bool wantReduced = (det.backLevel <= -3);
				if (wantReduced != speedReducedActive)
				{
					if (wantReduced) {
						asservdriver_->motion_setAccDecPercent(obstacleSpeedPercent_);
						logger().info() << "waitEndOfTraj BACKWARD: obstacle DETECTED -> setAccDecPercent "
								<< obstacleSpeedPercent_ << "%" << logs::end;
					} else {
						applySpeedSnapshotDirect(savedUserMaxSpeedActive,
								savedUserMaxSpeedPct, savedUserAccDecPercent);
						logger().info() << "waitEndOfTraj BACKWARD: obstacle CLEAR -> restore user (acc="
								<< savedUserAccDecPercent << "% pwmCap="
								<< (savedUserMaxSpeedActive ? savedUserMaxSpeedPct : 0)
								<< "%)" << logs::end;
					}
					speedReducedActive = wantReduced;
				}
			}
			// type == ROTATION : on ignore la détection
		}

		utils::sleep_for_micros(10000); // 10ms — laisse du CPU aux autres threads
		std::this_thread::yield();
		timeout++;
		if (timeout > 1000) // 10s timeout sécurité (1000 * 10ms)
		{
			logger().error() << "waitEndOfTrajWithDetection: TIMEOUT 10s "
					<< "(target_cmd=" << targetCmdId << " received_cmd=" << receivedCmdId
					<< " status=" << status << ")" << logs::end;
			return TRAJ_ERROR;
		}
	}
}

void Asserv::update_adv()
{
	logger().info() << "update_adv tob surcharged = " << logs::end;
}

int Asserv::getLowSpeedvalue()
{
	return lowSpeedvalue_;
}

void Asserv::setLowSpeedvalue(int value)
{
	lowSpeedvalue_ = value;
}

int Asserv::getMaxSpeedDistValue()
{
	return maxSpeedDistValue_;
}

void Asserv::setMaxSpeedDistValue(int value)
{
	maxSpeedDistValue_ = value;
}

// Callback de détection frontale pendant une trajectoire.
// Niveaux d'alerte :
TRAJ_STATE Asserv::goToChain(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(FORWARD,
				[this, x_match, yMM]() { asservdriver_->motion_GoToChain(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_GoToChain(x_match, yMM);
		return waitEndOfTrajWithDetection(FORWARD);
	}
	return TRAJ_ERROR;
}

TRAJ_STATE Asserv::goTo(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(FORWARD,
				[this, x_match, yMM]() { asservdriver_->motion_GoTo(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_GoTo(x_match, yMM);
		return waitEndOfTrajWithDetection(FORWARD);
	}
	return TRAJ_ERROR;
}

TRAJ_STATE Asserv::goBackTo(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(BACKWARD,
				[this, x_match, yMM]() { asservdriver_->motion_GoBackTo(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_GoBackTo(x_match, yMM);
		return waitEndOfTrajWithDetection(BACKWARD);
	}
	return TRAJ_ERROR;
}

TRAJ_STATE Asserv::goBackToChain(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(BACKWARD,
				[this, x_match, yMM]() { asservdriver_->motion_GoBackToChain(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_GoBackToChain(x_match, yMM);
		return waitEndOfTrajWithDetection(BACKWARD);
	}
	return TRAJ_ERROR;
}

void Asserv::setSimuSpeedMultiplier(float multiplier)
{
	if (asservdriver_ != nullptr)
		asservdriver_->setSimuSpeedMultiplier(multiplier);
}

// =============================================================================
// Envoi sans attente (pour mode CHAIN dans Navigator)
// =============================================================================

void Asserv::goToSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_GoTo(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_GoTo(x_match, yMM);
}

void Asserv::goToChainSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_GoToChain(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_GoToChain(x_match, yMM);
}

void Asserv::goBackToSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_GoBackTo(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_GoBackTo(x_match, yMM);
}

void Asserv::goBackToChainSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_GoBackToChain(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_GoBackToChain(x_match, yMM);
}

void Asserv::lineSend(float dist_mm)
{
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_Line(dist_mm);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_Line(dist_mm);
}

void Asserv::faceToSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_FaceTo(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_FaceTo(x_match, yMM);
}

void Asserv::faceBackToSend(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_FaceBackTo(x_match, yMM);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_FaceBackTo(x_match, yMM);
}

void Asserv::rotateRadSend(float radRelative)
{
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_RotateRad(radRelative);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_RotateRad(radRelative);
}

void Asserv::rotateDegSend(float degreesRelative)
{
	rotateRadSend(degToRad(degreesRelative));
}

void Asserv::rotateAbsDegSend(float thetaInDegreeAbsolute)
{
	float rad = changeMatchAngleRad(degToRad(thetaInDegreeAbsolute)) - pos_getTheta();
	rad = WrapAngle2PI(rad);
	rotateRadSend(rad);
}

void Asserv::orbitalTurnDegSend(float angleDeg, bool forward, bool turnRight)
{
	float rad = degToRad(angleDeg);
	if (useAsservType_ == ASSERV_EXT)
		asservdriver_->motion_OrbitalTurnRad(rad, forward, turnRight);
	else if (useAsservType_ == ASSERV_INT_ESIALR)
		pAsservEsialR_->motion_OrbitalTurnRad(rad, forward, turnRight);
}


// Avance ou recule en ligne droite.
// Le MovementType (FORWARD/BACKWARD) détermine quelle détection est active.
TRAJ_STATE Asserv::line(float dist_mm)
{
	const MovementType type = dist_mm > 0 ? FORWARD : BACKWARD;

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(type,
				[this, dist_mm]() { asservdriver_->motion_Line(dist_mm); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_Line(dist_mm);
		return waitEndOfTrajWithDetection(type);
	}
	return TRAJ_ERROR;
}

// Rotation relative en degrés : convertit en radians et délègue à rotateRad.
TRAJ_STATE Asserv::rotateDeg(float degreesRelative)
{
	return rotateRad(degToRad(degreesRelative));
}

// Rotation relative en radians. Bloquant.
// Les rotations ignorent la détection adversaire (MovementType::ROTATION).
TRAJ_STATE Asserv::rotateRad(float radiansRelative)
{
	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(ROTATION,
				[this, radiansRelative]() { asservdriver_->motion_RotateRad(radiansRelative); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_RotateRad(radiansRelative);
		return waitEndOfTrajWithDetection(ROTATION);
	}
	return TRAJ_ERROR;
}

//prend automatiquement un angle dans un sens ou dans l'autre suivant la couleur de match
TRAJ_STATE Asserv::rotateByMatchColorDeg(float thetaInDegreeRelative)
{
	if (matchColorPosition_ != 0)
	{
		return rotateDeg(-thetaInDegreeRelative); //couleur de match secondaire
	} else
		return rotateDeg(thetaInDegreeRelative); //couleur de match primaire
}

TRAJ_STATE Asserv::faceTo(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(ROTATION,
				[this, x_match, yMM]() { asservdriver_->motion_FaceTo(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_FaceTo(x_match, yMM);
		return waitEndOfTrajWithDetection(ROTATION);
	}
	return TRAJ_ERROR;
}

TRAJ_STATE Asserv::faceBackTo(float xMM, float yMM)
{
	float x_match = changeMatchX(xMM);

	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(ROTATION,
				[this, x_match, yMM]() { asservdriver_->motion_FaceBackTo(x_match, yMM); });
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_FaceBackTo(x_match, yMM);
		return waitEndOfTrajWithDetection(ROTATION);
	}
	return TRAJ_ERROR;
}

// Rotation absolue vers un angle donné sur le terrain.
// Calcul : angle_cible_converti - angle_courant = rotation relative nécessaire.
// La symétrie couleur est appliquée via changeMatchAngleRad.
// Le résultat est wrappé sur [-PI, PI] via WrapAngle2PI pour prendre le chemin le plus court.
TRAJ_STATE Asserv::rotateAbsDeg(float thetaInDegreeAbsolute)
{
	float rad = changeMatchAngleRad(degToRad(thetaInDegreeAbsolute)) - pos_getTheta();

	rad = WrapAngle2PI(rad);

	logger().debug() << "==== doRotateTo degrees=" << radToDeg(rad) << " thetaInDegreeAbsolute="
			<< thetaInDegreeAbsolute << logs::end;

	return rotateRad(rad);
}

// Avance vers un point (x,y) : 2 étapes.
// 1) Rotation absolue pour faire face au point (MovementType::ROTATION, detection bypass naturel)
// 2) line de la distance euclidienne calculée (+ adjustment_mm optionnel)
// Si déjà très proche du point (<5mm en dx ET dy), retourne directement TRAJ_FINISHED.
TRAJ_STATE Asserv::moveForwardTo(float xMM, float yMM, float adjustment_mm)
{
	// Snapshot coherent : un seul lock mutex pour calcul + log.
	ROBOTPOSITION p = pos_getPosition();
	float dx = changeMatchX(xMM) - p.x;
	float dy = yMM - p.y;
	if (std::abs(dx) < 5.0 && std::abs(dy) < 5.0)
	{
		logger().info() << "___ TRAJ_FINISHED __moveForwardTo (std::abs(dx) < 5.0 && std::abs(dy) < 5.0)"
				<< logs::end;
		return TRAJ_FINISHED;
	}
	float aRadian = atan2(dy, dx);

	aRadian = WrapAngle2PI(aRadian);

	logger().debug() << "moveForwardTo doRotateTo degrees=" << (aRadian * 180.0f) / M_PI << " dx=" << dx << " dy="
			<< dy << "  (aRadian * 180.0f) / M_PI)= " << (aRadian * 180.0f) / M_PI << " get="
			<< radToDeg(changeMatchAngleRad(aRadian)) << " xMM=" << xMM << " yMM=" << yMM << " getX=" << p.x
			<< " getY=" << p.y << logs::end;

	TRAJ_STATE ts = rotateAbsDeg(radToDeg(changeMatchAngleRad(aRadian)));
	if (ts != TRAJ_FINISHED)
		return ts;

	// Distance euclidienne calculee de facon numeriquement stable (cf asservchibios Goto).
	float dist = computeDeltaDist(dx, dy);
	logger().debug() << " __moveForwardTo dist=" << dist << logs::end;

	return line(dist + adjustment_mm);
}

TRAJ_STATE Asserv::moveBackwardTo(float xMM, float yMM)
{
	xMM = changeMatchX(xMM);

	// Snapshot coherent : un seul lock mutex.
	ROBOTPOSITION p = pos_getPosition();
	float dx = xMM - p.x;
	float dy = yMM - p.y;
	if (std::abs(dx) < 5.0 && std::abs(dy) < 5.0)
	{ //Augmenter les valeurs??? par rapport à l'asserv fenetre d'arrivée
		return TRAJ_FINISHED;
	}
	float aRadian = M_PI + atan2(dy, dx);
	aRadian = WrapAngle2PI(aRadian);

	TRAJ_STATE ts = rotateAbsDeg(radToDeg(changeMatchAngleRad(aRadian)));
	if (ts != TRAJ_FINISHED)
		return ts;

	// Distance euclidienne calculee de facon numeriquement stable (cf asservchibios Goto).
	float dist = computeDeltaDist(dx, dy);
	return line(-dist);
}

TRAJ_STATE Asserv::moveForwardAndRotateTo(float xMM, float yMM, float thetaInDegree)
{
	TRAJ_STATE ts = moveForwardTo(xMM, yMM);
	if (ts != TRAJ_FINISHED) return ts;

	return rotateAbsDeg(thetaInDegree);
}
TRAJ_STATE Asserv::moveBackwardAndRotateTo(float xMM, float yMM, float thetaInDegree)
{
	TRAJ_STATE ts;
	ts = moveBackwardTo(xMM, yMM);
	if (ts != TRAJ_FINISHED) return ts;
	ts = rotateAbsDeg(thetaInDegree);
	return ts;
}

std::tuple<int, float, float> Asserv::eq_2CirclesCrossed_getXY(float x1, float y1, float d1, float x2, float y2,
		float d2, float robot_size_l_mm)
{
	logger().debug() << "eq_2CirclesCrossed_getXY x1=" << x1 << " y1=" << y1 << " d1=" << d1 << "  x2=" << x2 << " y2="
			<< y2 << " d2=" << d2 << logs::end;
//On définit y en fonction de x : y = ax +b
	float b = (((x2 * x2) + (y2 * y2) + (d1 * d1) - (d2 * d2) - (x1 * x1) - (y1 * y1)) / (2.0 * (y2 - y1)));
	float a = (x2 - x1) / (y2 - y1);

//resolution de l'equation du second degré Ax² + Bx + C = 0
	float A = (a * a) + 1.0;
	float B = ((2.0 * y1 * a) - (2 * x1) - (2.0 * a * b));
	float C = ((x1 * x1) + (y1 * y1) - (2.0 * y1 * b) + (b * b) - (d1 * d1));

	return eq_2nd_deg_getXY(a, b, A, B, C, robot_size_l_mm);

}

float Asserv::eq_2nd_deg_getDelta(float A, float B, float C)
{
	return ((B * B) - (4.0 * A * C));
}

std::tuple<int, float, float> Asserv::eq_2nd_deg_getXY(float a, float b, float A, float B, float C,
		float robot_size_l_mm)
{
	logger().debug() << "eq_2nd_deg_getXY a=" << a << " b=" << b << " A=" << A << " B=" << B << " C=" << C << logs::end;
	float coord_x = 0.0, coord_y = 0.0;
	float delta = eq_2nd_deg_getDelta(A, B, C);
	if (delta < 0)
	{
		return std::make_tuple(0, -1, -1);
	}
	if (delta == 0)
	{
		coord_x = (-1.0 * B / (2.0 * A));
		coord_y = b - (a * coord_x);

		return std::make_tuple(1, coord_x, coord_y);
	}
	if (delta > 0)
	{
		float x1 = (((-1.0 * B) + std::sqrt(delta)) / (2.0 * A));
		float y1 = b - (a * x1); //b- ax
		float x2 = ((-1.0 * B - std::sqrt(delta)) / (2.0 * A));
		float y2 = b - (a * x2); //b- ax

		logger().debug() << " solutions : x1=" << x1 << " y1=" << y1 << "   x2=" << x2 << " y2=" << y2 << logs::end;
		//filtrage de la table et de la largeur du robot !!
		if (y1 < robot_size_l_mm && y2 > robot_size_l_mm)
		{
			coord_x = x2;
			coord_y = y2;
			return std::make_tuple(2, coord_x, coord_y);
		} else if (y2 < robot_size_l_mm && y1 > robot_size_l_mm)
		{
			coord_x = x1;
			coord_y = y1;
			return std::make_tuple(3, coord_x, coord_y);
		} else
		{

			//double solution impossible à déterminer
			return std::make_tuple(-2, -1, -1);
		}
	} else
		return std::make_tuple(-1, -1, -1);

}

int Asserv::adjustRealPosition(float pos_x_start_mm, float pos_y_start_mm, ROBOTPOSITION p, float delta_jx_mm,
		float delta_ky_mm, float mesure_mm, float robot_size_l_mm)
{

	float pos_x_start_mm_conv = changeMatchX(pos_x_start_mm);
	logger().debug() << "adjustRealPosition : pos_x_start_mm=" << pos_x_start_mm_conv << " pos_y_start_mm="
			<< pos_y_start_mm << " p.x=" << p.x << " p.y=" << p.y << " p.theta=" << p.theta << " degrees="
			<< p.theta * 180 / M_PI << " delta_jx_mm=" << delta_jx_mm << " delta_ky_mm=" << delta_ky_mm << " mesure_mm="
			<< mesure_mm << logs::end;

	float dist_real_mm = std::sqrt(
			(((p.x) - pos_x_start_mm_conv) * ((p.x) - pos_x_start_mm_conv))
					+ (((p.y) - pos_y_start_mm) * ((p.y) - pos_y_start_mm)));

	float dist_x_when_mesuring_mm = std::abs(p.x - pos_x_start_mm_conv);
	float position_rel_theta_when_mesuring_rad = changeMatchAngleRad(p.theta); //on cherche juste l'angle relatif qu'on soit en couleur A ou B

//calcul de l'angle entre les 2 rayons de cercle = angle_rad
	float alphap_rad = acos((dist_x_when_mesuring_mm / dist_real_mm));
	float gamma_rad = atan2(delta_jx_mm, (delta_ky_mm + mesure_mm));
	float angle_rad = M_PI_2 + alphap_rad + gamma_rad + position_rel_theta_when_mesuring_rad;

//calcul de BCprim (distance centre robot au point de mesure)
	float BCprim_mm = std::sqrt((delta_jx_mm * delta_jx_mm) + ((delta_ky_mm + mesure_mm) * (delta_ky_mm + mesure_mm)));

//calcul de la distance entre les 2 centres de cercle DCprim
	float DCprim_mm = std::sqrt(
			(dist_real_mm * dist_real_mm) + (BCprim_mm * BCprim_mm)
					- (2.0 * dist_real_mm * BCprim_mm * cos(angle_rad)));

//calcul de l'abcisse du 2eme centre de cercle par rapport au premier
	float dist_xDCp_mm = std::sqrt((DCprim_mm * DCprim_mm) - (pos_y_start_mm * pos_y_start_mm));

//on determine le croisement des 2 cercles en fct des coord et des rayons
	auto r = eq_2CirclesCrossed_getXY(pos_x_start_mm_conv, pos_y_start_mm, dist_real_mm,
			changeMatchX(pos_x_start_mm, dist_xDCp_mm), 0.0, BCprim_mm, robot_size_l_mm);

	float err = std::get<0>(r);

	logger().debug() << "alphap_rad=" << alphap_rad << " deg=" << alphap_rad * 180 / M_PI << " gamma_rad=" << gamma_rad
			<< " deg=" << gamma_rad * 180 / M_PI << " angle_rad=" << angle_rad << " deg=" << angle_rad * 180 / M_PI
			<< " position_rel_theta_when_mesuring_rad=" << position_rel_theta_when_mesuring_rad
			<< " dist_x_when_mesuring_mm=" << dist_x_when_mesuring_mm << " dist_real_mm=" << dist_real_mm << " BCprim="
			<< BCprim_mm << " DCprim=" << DCprim_mm << " dist_xDCprim_mm=" << dist_xDCp_mm << " err=" << err
			<< " new_x_mm=" << std::get<1>(r) << " new_y_mm=" << std::get<2>(r) << logs::end;

	if (err <= 0) return err;

	float new_x_mm = std::get<1>(r);
	float new_y_mm = std::get<2>(r);

//calcul de l'angle de correction
//ajouter la difference entre ancien alphap et le nouveau calculé à la position Theta
	float new_alphap = std::acos(std::abs((new_x_mm - pos_x_start_mm_conv)) / dist_real_mm);

//float new_teta = getRelativeAngle((position_rel_theta_when_mesuring_rad + (alphap_rad - new_alphap)) * 180.0 / M_PI) * M_PI / 180.0;

//TODO essaie de correction de l'angle
	float new_teta = changeMatchAngleRad((position_rel_theta_when_mesuring_rad - (alphap_rad - new_alphap)));

	logger().debug() << "new pos : x=" << new_x_mm << " y=" << new_y_mm << " a=" << new_teta << " degrees="
			<< new_teta * 180 / M_PI << logs::end;
//set de la nouvelle position et angle
	setPositionReal(new_x_mm, new_y_mm, new_teta);

	return 1; // il y a un resultat viable.
}

bool Asserv::calculateDriftRightSideAndSetPos(float d2_theo_bordure_mm, float d2b_bordure_mm, float x_depart_mm,
		float y_depart_mm)
{
	logger().error() << "calculate : " << " d2_theo_bordure_mm= " << d2_theo_bordure_mm << " d2b_bordure_mm= "
			<< d2b_bordure_mm << " x_depart_mm= " << x_depart_mm << " y_depart_mm= " << y_depart_mm << logs::end;

	if (abs(d2b_bordure_mm - d2_theo_bordure_mm) >= 5)
	{

//Partie théorique basé sur la position d'arrivée (que croit le robot)
		ROBOTPOSITION p = pos_getPosition();
		//tan teta = d1/l
		float dx = (p.x) - x_depart_mm;
		float dy = (p.y) - y_depart_mm;
		float l_theo_mm = std::sqrt(dx * dx + dy * dy);
		float d1_theo = d2_theo_bordure_mm; //TODO a modifier avec cos(beta)
		float teta_theo = atan2(d1_theo, l_theo_mm);

		logger().error() << "calculate : " << " dx= " << dx << " dy= " << dy << " l_theo_mm= " << l_theo_mm
				<< " teta_theo_deg= " << teta_theo * 180.0 / M_PI << logs::end;

//partie reelle (avec la distance mesurée de la bordure
		//tan teta = d1b/l
		float d1_b = d2b_bordure_mm;    //TODO a modifier avec cos(beta)
		float teta_b = atan2(d1_b, l_theo_mm);

		float teta_error = teta_b - teta_theo;
		logger().error() << "calculate  : " << " teta_b_deg= " << teta_b * 180.0 / M_PI << " teta_error_deg= "
				<< teta_error * 180.0 / M_PI << " p.x=" << p.x << " l_theo_mm =" << (l_theo_mm) << logs::end;

		float alpha = acos((float) ((dx) / (l_theo_mm)));
		float alpha_error = alpha - teta_error;

		logger().error() << "calculate  : " << " teta_b= " << teta_b << " teta_error= " << teta_error << " alpha_deg= "
				<< alpha * 180.0 / M_PI << " alpha_error_deg= " << alpha_error * 180.0 / M_PI << logs::end;
		//changement des coordonnées
		float new_teta = p.theta + teta_error;
		float new_x = x_depart_mm + (l_theo_mm * cos(alpha_error));
		float new_y = y_depart_mm - (l_theo_mm * sin(alpha_error));

		logger().error() << "old position : " << " x= " << p.x << " y= " << p.y << " a_deg= " << p.theta * 180.0 / M_PI
				<< logs::end;
		//pour 2 deg on a un decalage de 10mm
		float x_corr = (abs(teta_error) * 180.0 / M_PI) * 10.0 / 2.0;
		logger().error() << "x_corr : " << x_corr << logs::end;
		new_x = new_x + x_corr;

		logger().error() << "setPosition  : " << " x= " << new_x << " y= " << new_y << " a_rad= " << new_teta
				<< " a_deg= " << new_teta * 180.0 / M_PI << logs::end;
		//regle de 3 pour modifier le x (decalage sur aire de depart qui influe sur le x)

		setPositionAndColor(new_x, new_y, new_teta * 180.0 / M_PI, matchColorPosition_);
		return true;
	} else
		return false;

}

bool Asserv::calculateDriftLeftSideAndSetPos(float d2_theo_bordure_mm, float d2b_bordure_mm, float x_depart_mm,
		float y_depart_mm)
{
	logger().error() << "calculate : " << " d2_theo_bordure_mm= " << d2_theo_bordure_mm << " d2b_bordure_mm= "
			<< d2b_bordure_mm << " x_depart_mm= " << changeMatchX(x_depart_mm) << " y_depart_mm= " << y_depart_mm
			<< logs::end;

	x_depart_mm = changeMatchX(x_depart_mm);

	if (abs(d2b_bordure_mm - d2_theo_bordure_mm) >= 5)
	{

//Partie théorique basé sur la position d'arrivée (que croit le robot)
		ROBOTPOSITION p = pos_getPosition();
		//tan teta = d1/l
		//float dx = (p.x * 1000.0) - getRelativeX(x_depart_mm);
		float dx = (p.x) - x_depart_mm;
		float dy = (p.y) - y_depart_mm;
		float l_theo_mm = std::sqrt(dx * dx + dy * dy);
		float d1_theo = d2_theo_bordure_mm; //TODO a modifier avec cos(beta)
		float teta_theo = atan2(d1_theo, l_theo_mm);

		logger().error() << "calculate : " << " dx= " << dx << " dy= " << dy << " l_theo_mm= " << l_theo_mm
				<< " teta_theo_deg= " << teta_theo * 180.0 / M_PI << logs::end;

//partie reelle (avec la distance mesurée de la bordure
		//tan teta = d1b/l
		float d1_b = d2b_bordure_mm;    //TODO a modifier avec cos(beta)
		float teta_b = atan2(d1_b, l_theo_mm);

		float teta_error = teta_b - teta_theo;
		logger().error() << "calculate  : " << " teta_b_deg= " << teta_b * 180.0 / M_PI << " teta_error_deg= "
				<< teta_error * 180.0 / M_PI << " p.x=" << p.x << " l_theo_mm=" << (l_theo_mm) << logs::end;

		float alpha = acos((float) ((abs(dx)) / (l_theo_mm)));    //yellow
		float alpha_error = alpha - teta_error;

		logger().error() << "calculate  : " << " teta_b= " << teta_b << " teta_error= " << teta_error << " alpha_deg= "
				<< alpha * 180.0 / M_PI << " alpha_error_deg= " << alpha_error * 180.0 / M_PI << logs::end;
		//changement des coordonnées
		float new_teta = (p.theta - teta_error);
		//float new_x = getRelativeX(x_depart_mm) - (l_theo_mm * cos(alpha_error)); //yellow
		float new_x = x_depart_mm - (l_theo_mm * cos(alpha_error)); //yellow
		float new_y = y_depart_mm - (l_theo_mm * sin(alpha_error));

		logger().error() << "old position : " << " x= " << p.x << " y= " << p.y << " a_deg= " << p.theta * 180.0 / M_PI
				<< logs::end;
		//pour 2 deg on a un decalage de 10mm
		float x_corr = (abs(teta_error) * 180.0 / M_PI) * 10.0 / 2.0;
		logger().error() << "x_corr : " << x_corr << logs::end;
		new_x = new_x + x_corr;

		logger().error() << "setPosition  : " << " x= " << new_x << " y= " << new_y << " a_rad= " << new_teta
				<< " a_deg= " << new_teta * 180.0 / M_PI << logs::end;
		//regle de 3 pour modifier le x (decalage sur aire de depart qui influe sur le x)

		setPositionAndColor(changeMatchX(new_x), new_y, radToDeg(changeMatchAngleRad(new_teta)), matchColorPosition_);

		return true;
	} else
		return false;

}

// Rotation orbitale asservie autour d'une roue.
// MovementType ROTATION : la trajectoire courbe peut intersecter la zone
// adversaire mais on suppose ici qu'elle est preparee a l'avance (rare en
// match). On reste sur le meme choix que rotateRad/faceTo.
TRAJ_STATE Asserv::orbitalTurnDeg(float angleDeg, bool forward, bool turnRight)
{
	float rad = degToRad(angleDeg);
	if (useAsservType_ == ASSERV_EXT)
		return sendCborMotionWithRetry(ROTATION,
				[this, rad, forward, turnRight]() {
					asservdriver_->motion_OrbitalTurnRad(rad, forward, turnRight);
				});
	else if (useAsservType_ == ASSERV_INT_ESIALR) {
		pAsservEsialR_->motion_OrbitalTurnRad(rad, forward, turnRight);
		return pAsservEsialR_->waitEndOfTraj();
	}
	return TRAJ_ERROR;
}

// Pivot autour de la roue gauche : désactive la régulation, actionne les moteurs,
// attend la durée, puis réactive la régulation et le maintien en position.
void Asserv::pivotLeft(int powerL, int powerR, int timems)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->motion_FreeMotion();
		pAsservEsialR_->motion_ActivateReguAngle(false);
		pAsservEsialR_->motion_ActivateReguDist(false);
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();

		runMotorRight(powerR, timems);
		runMotorLeft(powerL, timems);
		utils::sleep_for_micros(timems * 1000);

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		pAsservEsialR_->motion_AssistedHandling();
		resetEmergencyOnTraj("pivotLeft");
	} else if (useAsservType_ == ASSERV_EXT)
	{

		//TODO
	}
}

// Pivot autour de la roue droite : même principe que pivotLeft.
void Asserv::pivotRight(int powerL, int powerR, int timems)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		pAsservEsialR_->motion_FreeMotion();
		pAsservEsialR_->motion_ActivateReguAngle(false);
		pAsservEsialR_->motion_ActivateReguDist(false);
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();

		runMotorRight(powerR, timems);
		runMotorLeft(powerL, timems);
		utils::sleep_for_micros(timems * 1000);

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		pAsservEsialR_->motion_AssistedHandling();
		resetEmergencyOnTraj("pivotRight");
	} else if (useAsservType_ == ASSERV_EXT)
	{

		//TODO
	}
}

TRAJ_STATE Asserv::calage2(int distmm, int percent)
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		if (distmm > 0)
			pAsservEsialR_->motion_setLowSpeedForward(true, percent);
		else if (distmm < 0) pAsservEsialR_->motion_setLowSpeedBackward(true, percent);

		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();

		pAsservEsialR_->motion_AssistedHandling();
		TRAJ_STATE ts = TRAJ_IDLE;
		while (ts == TRAJ_IDLE)
		{
			{ pAsservEsialR_->motion_DoDirectLine(distmm); ts = pAsservEsialR_->waitEndOfTraj(); } //sans asservissement L/R
			std::this_thread::yield();
		}

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(false);
		pAsservEsialR_->motion_ActivateReguDist(true);
		resetEmergencyOnTraj("doCalage phase 1");

		{ pAsservEsialR_->motion_DoDirectLine(distmm); ts = pAsservEsialR_->waitEndOfTraj(); } //sans asservissement L/R

		//pAsservEsialR_->path_CancelTrajectory();
		pAsservEsialR_->motion_setLowSpeedForward(false);
		pAsservEsialR_->motion_setLowSpeedBackward(false);

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		resetEmergencyOnTraj("doCalage phase 2");

		return ts;
	} else if (useAsservType_ == ASSERV_EXT)
	{

		//TODO A finir
	}
	return TRAJ_ERROR;
}

TRAJ_STATE Asserv::calage(int distmm, int percent)
{

	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		if (distmm > 0)
			pAsservEsialR_->motion_setLowSpeedForward(true, percent);
		else if (distmm < 0) pAsservEsialR_->motion_setLowSpeedBackward(true, percent);

		pAsservEsialR_->motion_ActivateReguAngle(false);
		pAsservEsialR_->motion_ActivateReguDist(true);
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();

		pAsservEsialR_->motion_AssistedHandling();
		//TRAJ_STATE ts = TRAJ_OK;
		pAsservEsialR_->motion_DoDirectLine(distmm); TRAJ_STATE ts = pAsservEsialR_->waitEndOfTraj(); //sans asservissement L/R

		//pAsservEsialR_->path_CancelTrajectory();
		pAsservEsialR_->motion_setLowSpeedForward(false);
		pAsservEsialR_->motion_setLowSpeedBackward(false);

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		resetEmergencyOnTraj("doCalage");

		return ts;
	} else if (useAsservType_ == ASSERV_EXT)
	{

		if (distmm > 0)
			asservdriver_->motion_setLowSpeedForward(true, percent);
		else if (distmm < 0) asservdriver_->motion_setLowSpeedBackward(true, percent);

//        asservdriver_->motion_ActivateReguAngle(false);
//        asservdriver_->motion_ActivateReguDist(true);
		asservdriver_->motion_AssistedHandling();

		asservdriver_->motion_Line(distmm); TRAJ_STATE ts = asservdriver_->waitEndOfTraj(); //sans asservissement L/R

		asservdriver_->motion_setLowSpeedForward(false);
		asservdriver_->motion_setLowSpeedBackward(false);

		asservdriver_->motion_ActivateReguAngle(true);
		asservdriver_->motion_ActivateReguDist(true);
		resetEmergencyOnTraj("doCalage");

		return ts;
	} else
		return TRAJ_ERROR;
}

TRAJ_STATE Asserv::calageNew(float dist_mm, int percent, float timeout_ms) // if distance <0, move backward
{
	if (useAsservType_ == ASSERV_INT_ESIALR)
	{
		if (dist_mm > 0)
			pAsservEsialR_->motion_setLowSpeedForward(true, percent);
		else if (dist_mm < 0) pAsservEsialR_->motion_setLowSpeedBackward(true, percent);

		pAsservEsialR_->motion_ActivateReguAngle(false);
		pAsservEsialR_->motion_ActivateReguDist(true);
		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();

		pAsservEsialR_->motion_AssistedHandling();

		//TODO implementer le timeout ??

		pAsservEsialR_->motion_DoDirectLine(dist_mm); TRAJ_STATE ts = pAsservEsialR_->waitEndOfTraj(); //sans asservissement L/R

		//pAsservEsialR_->path_CancelTrajectory();
		pAsservEsialR_->motion_setLowSpeedForward(false);
		pAsservEsialR_->motion_setLowSpeedBackward(false);

		pAsservEsialR_->motion_ResetReguAngle();
		pAsservEsialR_->motion_ResetReguDist();
		pAsservEsialR_->motion_ActivateReguAngle(true);
		pAsservEsialR_->motion_ActivateReguDist(true);
		resetEmergencyOnTraj("doCalage");

		return ts;
	} else if (useAsservType_ == ASSERV_EXT)
	{

		if (dist_mm > 0)
			asservdriver_->motion_setLowSpeedForward(true, percent);
		else if (dist_mm < 0) asservdriver_->motion_setLowSpeedBackward(true, percent);

        asservdriver_->motion_ActivateReguAngle(false);
        asservdriver_->motion_ActivateReguDist(true);

        //TODO implementer le timeout ??
		asservdriver_->motion_Line(dist_mm); TRAJ_STATE ts = asservdriver_->waitEndOfTraj();

		asservdriver_->motion_setLowSpeedForward(false);
		asservdriver_->motion_setLowSpeedBackward(false);

		asservdriver_->motion_ActivateReguAngle(true);
		asservdriver_->motion_ActivateReguDist(true);
		//resetEmergencyOnTraj("doCalage");

		return ts;
	} else
		return TRAJ_ERROR;
}



//FONCTION ASSERV DE BASE

void Asserv::resetEncoders()
{
	asservdriver_->resetEncoders();
}
void Asserv::getDeltaEncodersCounts(int *deltaCountR, int *deltaCountL)
{
	asservdriver_->getDeltaCountsExternal(deltaCountR, deltaCountL);
}
void Asserv::getEncodersCounts(int *countR, int *countL)
{
	asservdriver_->getCountsExternal(countR, countL);
}
void Asserv::runMotorLeft(int power, int timems)
{
	asservdriver_->setMotorLeftPower(power, timems);
}
void Asserv::runMotorRight(int power, int timems)
{
	asservdriver_->setMotorRightPower(power, timems);
}
void Asserv::stopMotors()
{
	asservdriver_->stopMotors();
}
