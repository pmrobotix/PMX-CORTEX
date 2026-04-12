/*!
 * \file
 * \brief Evenement de detection d'obstacle.
 *
 * Structure partagee entre Sensors (producteur) et Navigator/IA (consommateurs).
 * Inclut beacon_delay_us et beacon_seq pour le debug de synchronisation.
 * Contient la position de l'adversaire detecte, le niveau de menace,
 * et le timestamp de la detection pour la synchronisation.
 *
 * Logique pure, header-only, aucune dependance driver.
 */

#ifndef COMMON_GEOMETRY_DETECTIONEVENT_HPP_
#define COMMON_GEOMETRY_DETECTIONEVENT_HPP_

#include <cstdint>

/*!
 * \brief Evenement de detection d'un obstacle/adversaire.
 *
 * Publie par SensorsThread::sensorOnTimer() apres chaque cycle de detection.
 * Lu par Navigator pour decider retry vs contournement.
 */
struct DetectionEvent {

	// --- Niveaux de detection ---

	int frontLevel;   ///< 0=rien, 1=droite, 2=gauche, 3=moyen, 4=dead center (STOP)
	int backLevel;    ///< 0=rien, -1=droite, -2=gauche, -3=moyen, -4=dead center (STOP)

	// --- Position adversaire detecte (coordonnees table, mm) ---

	float x_adv_mm;   ///< Position X adversaire sur la table (-1 si inconnu).
	float y_adv_mm;   ///< Position Y adversaire sur la table (-1 si inconnu).
	float d_adv_mm;    ///< Distance robot-adversaire en mm (-1 si inconnu).

	// --- Position robot au moment de la detection ---

	float x_robot_mm;  ///< Position X robot au moment du sync beacon.
	float y_robot_mm;  ///< Position Y robot au moment du sync beacon.
	float theta_robot_rad; ///< Angle robot au moment du sync beacon.

	// --- Timing beacon (debug/synchronisation) ---

	uint16_t beacon_delay_us; ///< Delta mesure beacon du robot detecte (us depuis debut cycle Teensy).
	uint32_t beacon_seq;      ///< Numero de sequence beacon (debug, incremente chaque cycle).

	// --- Metadata ---

	uint64_t timestamp_us;  ///< Timestamp de la detection (µs depuis demarrage chrono).
	bool valid;             ///< true si une detection recente est presente.

	/*!
	 * \brief Constructeur par defaut : aucune detection.
	 */
	DetectionEvent()
		: frontLevel(0), backLevel(0),
		  x_adv_mm(-1.0f), y_adv_mm(-1.0f), d_adv_mm(-1.0f),
		  x_robot_mm(0.0f), y_robot_mm(0.0f), theta_robot_rad(0.0f),
		  beacon_delay_us(0), beacon_seq(0),
		  timestamp_us(0), valid(false)
	{
	}

	/*!
	 * \brief Reinitialise l'evenement (aucune detection).
	 */
	void clear()
	{
		frontLevel = 0;
		backLevel = 0;
		x_adv_mm = -1.0f;
		y_adv_mm = -1.0f;
		d_adv_mm = -1.0f;
		x_robot_mm = 0.0f;
		y_robot_mm = 0.0f;
		theta_robot_rad = 0.0f;
		beacon_delay_us = 0;
		beacon_seq = 0;
		timestamp_us = 0;
		valid = false;
	}

	/*!
	 * \brief Teste si un obstacle bloquant est detecte (level 4 ou -4).
	 */
	inline bool isBlocking() const
	{
		return (frontLevel == 4 || backLevel == -4);
	}

	/*!
	 * \brief Teste si un ralentissement est necessaire (level >= 3 ou <= -3).
	 */
	inline bool isSlowDown() const
	{
		return (frontLevel >= 3 || backLevel <= -3);
	}

	/*!
	 * \brief Teste si la position adversaire est connue.
	 */
	inline bool hasPosition() const
	{
		return (valid && x_adv_mm >= 0.0f && y_adv_mm >= 0.0f);
	}
};

#endif
