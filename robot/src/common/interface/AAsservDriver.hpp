/*!
 * \file
 * \brief Interface abstraite du driver d'asservissement.
 *
 * Definit les commandes de mouvement haut niveau (goto, rotation, ligne)
 * et le controle bas niveau des moteurs/encodeurs. Les implementations
 * concretes sont dans driver-arm/ (carte ST serie), driver-simu/ (simulation)
 * et asserv-esial/ (PID interne).
 */

#ifndef COMMON_ASSERVDRIVER_AASSERVDRIVER_HPP_
#define COMMON_ASSERVDRIVER_AASSERVDRIVER_HPP_

#include <cstdint>
#include <string>

#include "ARobotPositionShared.hpp"

/*!
 * \brief Etat de retour d'une trajectoire.
 */
enum TRAJ_STATE {
	TRAJ_IDLE,                      ///< Etat initial avant trajectoire.
	TRAJ_FINISHED,                  ///< Trajectoire terminee avec succes.

	TRAJ_INTERRUPTED = 10,          ///< Trajectoire interrompue par le logiciel.
	TRAJ_IMPOSSIBLE = 11,           ///< Trajectoire annulee ou chemin introuvable.
	TRAJ_NEAR_OBSTACLE = 100,       ///< Arret pour obstacle proche (capteurs).
	TRAJ_COLLISION = 101,           ///< Arret pour collision (asserv bloque).

	TRAJ_REAR_OBSTACLE = 200,       ///< \deprecated Obstacle arriere.
	TRAJ_REAR_COLLISION = 201,      ///< \deprecated Collision arriere.

	TRAJ_ERROR = 999                ///< Erreur generique.
};

/*!
 * \brief Etat courant du mode de deplacement.
 */
enum MOTION_STATE {
	TRAJECTORY_RUNNING,     ///< Trajectoire en cours d'execution.
	ASSISTED_HANDLING,      ///< Mode deplacement assiste (PID actif).
	FREE_MOTION,            ///< Roue libre, pas d'asservissement.
	DISABLE_PID,            ///< PID desactive, moteurs stoppes.
};

/*!
 * \brief Interface abstraite du driver d'asservissement.
 *
 * Regroupe les commandes de mouvement (goto, rotation, ligne),
 * le controle direct des moteurs/encodeurs, et la gestion de l'odometrie.
 * Chaque plateforme (ARM, SIMU, EsialR) fournit sa propre implementation.
 *
 * \note Cette interface contient 47 methodes et sera a terme decoupee
 *       en AAsserv (haut niveau) + AMotorDriver (bas niveau).
 *       Voir ARCHITECTURE.md pour le plan de refactoring.
 */
class AAsservDriver {

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botid Identifiant du robot (pour le fichier de config).
	 * \param aRobotPositionShared Pointeur vers la position partagee du robot.
	 * \return Pointeur vers l'instance creee (ARM, SIMU ou EsialR).
	 */
	static AAsservDriver* create(std::string botid, ARobotPositionShared *aRobotPositionShared);

	virtual ~AAsservDriver()
	{
	}

	AAsservDriver()
	{
	}

	/*!
	 * \brief Verifie si le driver d'asservissement est connecte.
	 * \return true si la communication est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Actions de nettoyage avant l'arret du programme.
	 */
	virtual void endWhatTodo() = 0;

	// ---- Controle direct moteurs (bas niveau) ----
	// Note: setMotorLeftPower/RightPower, stopMotors, getCountsExternal
	// sont gardes car utilises par des tests O_* et Asserv.cpp.
	// Les autres methodes deprecated ont ete supprimees (dead code).

	virtual void setMotorLeftPower(int power, int time_ms) = 0;
	virtual void setMotorRightPower(int power, int time_ms) = 0;
	virtual void stopMotors() = 0;
	virtual void getCountsExternal(int32_t *countR, int32_t *countL) = 0;
	virtual void getDeltaCountsExternal(int32_t *deltaR, int32_t *deltaL) = 0;
	virtual void resetEncoders() = 0;

	// ---- Asservissement externe (haut niveau) ----

	/*!
	 * \brief Active ou desactive le thread d'asservissement.
	 * \param enablethread true pour demarrer, false pour arreter.
	 */
	virtual void motion_ActivateManager(bool enablethread) = 0;

	/*!
	 * \brief Definit la position courante du robot (recalage).
	 * \param x_mm Position X en millimetres.
	 * \param y_mm Position Y en millimetres.
	 * \param angle_rad Angle en radians.
	 */
	virtual void odo_SetPosition(float x_mm, float y_mm, float angle_rad) = 0;

	/*!
	 * \brief Lit la position courante du robot.
	 * \return Structure ROBOTPOSITION (x en mm, y en mm, theta en radians).
	 */
	virtual ROBOTPOSITION odo_GetPosition() = 0;

	/*!
	 * \brief Interrompt la trajectoire en cours (arret d'urgence logiciel).
	 */
	virtual void emergencyStop() = 0;

	/*!
	 * \brief Reinitialise l'arret d'urgence pour permettre un nouveau mouvement.
	 */
	virtual void resetEmergencyStop() = 0;

	// ---- Commandes de mouvement ----

	/*!
	 * \brief Oriente le robot face a un point.
	 * \param x_mm Coordonnee X cible en mm.
	 * \param y_mm Coordonnee Y cible en mm.
	 * \param back_face true pour faire face avec l'arriere du robot.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_DoFace(float x_mm, float y_mm, bool back_face) = 0;

	/*!
	 * \brief Avance ou recule en ligne droite.
	 * \param dist_mm Distance en mm (positif = avant, negatif = arriere).
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_DoLine(float dist_mm) = 0;

	/*!
	 * \brief Effectue une rotation sur place.
	 * \param angle_radians Angle de rotation en radians.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_DoRotate(float angle_radians) = 0;

	/*!
	 * \brief Effectue une rotation orbitale (pivot sur une roue).
	 * \param angle_radians Angle de rotation en radians.
	 * \param forward true = marche avant, false = marche arriere.
	 * \param turnRight true = pivot roue droite, false = pivot roue gauche.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_DoOrbitalTurn(float angle_radians, bool forward, bool turnRight) = 0;

	/*!
	 * \brief Deplace le robot vers un point (marche avant).
	 * \param x_mm Coordonnee X cible en mm.
	 * \param y_mm Coordonnee Y cible en mm.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_Goto(float x_mm, float y_mm) = 0;

	/*!
	 * \brief Deplace le robot vers un point (marche arriere).
	 * \param x_mm Coordonnee X cible en mm.
	 * \param y_mm Coordonnee Y cible en mm.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_GotoReverse(float x_mm, float y_mm) = 0;

	/*!
	 * \brief Deplace le robot vers un point sans s'arreter (enchainement).
	 * \param x_mm Coordonnee X cible en mm.
	 * \param y_mm Coordonnee Y cible en mm.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_GotoChain(float x_mm, float y_mm) = 0;

	/*!
	 * \brief Deplace le robot vers un point en marche arriere sans s'arreter.
	 * \param x_mm Coordonnee X cible en mm.
	 * \param y_mm Coordonnee Y cible en mm.
	 * \return Etat de la trajectoire a la fin du mouvement.
	 */
	virtual TRAJ_STATE motion_GotoReverseChain(float x_mm, float y_mm) = 0;

	// ---- Controle du mode de deplacement ----

	/*!
	 * \brief Passe en roue libre (aucun asservissement).
	 */
	virtual void motion_FreeMotion(void) = 0;

	/*!
	 * \brief Active le mode deplacement assiste (PID actif).
	 */
	virtual void motion_AssistedHandling(void) = 0;

	// ---- Reglage de vitesse ----

	/*!
	 * \brief Active/desactive la vitesse reduite en marche avant.
	 * \param enable true pour activer.
	 * \param percent Pourcentage de la vitesse max (0-100).
	 */
	virtual void motion_setLowSpeedForward(bool enable, int percent = 0) = 0;

	/*!
	 * \brief Active/desactive la vitesse reduite en marche arriere.
	 * \param enable true pour activer.
	 * \param percent Pourcentage de la vitesse max (0-100).
	 */
	virtual void motion_setLowSpeedBackward(bool enable, int percent = 0) = 0;

	/*!
	 * \brief Definit la vitesse maximale.
	 * \param enable true pour activer la limitation.
	 * \param speed_dist_m_sec Vitesse lineaire max en mm/s.
	 * \param speed_angle_rad_sec Vitesse angulaire max en rad/s.
	 */
	virtual void motion_setMaxSpeed(bool enable, int speed_dist_m_sec = 0, int speed_angle_rad_sec = 0) = 0;

	// ---- Regulation (deprecated) ----

	/*!
	 * \brief Active/desactive la regulation en distance.
	 * \param enable true pour activer.
	 * \deprecated Utilise uniquement par ARM et EsialR.
	 */
	virtual void motion_ActivateReguDist(bool enable) = 0;

	/*!
	 * \brief Active/desactive la regulation en angle.
	 * \param enable true pour activer.
	 * \deprecated Utilise uniquement par ARM et EsialR.
	 */
	virtual void motion_ActivateReguAngle(bool enable) = 0;

};

#endif
