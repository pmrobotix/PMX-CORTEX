/*!
 * \file
 * \brief Classification des obstacles par zones autour du robot.
 *
 * Logique pure de detection : configuration des capteurs (enable/ignore),
 * seuils de distance, et filtres de classification (level 0-4).
 * Aucune dependance driver ou robot — testable en isolation.
 *
 * Extraite de Sensors (ancien PMX) pour separer la logique pure
 * de l'orchestration driver/timer.
 */

#ifndef COMMON_GEOMETRY_OBSTACLEZONE_HPP_
#define COMMON_GEOMETRY_OBSTACLEZONE_HPP_

#include "log/LoggerFactory.hpp"

/*!
 * \brief Classification des obstacles par zones autour du robot.
 *
 * Zones de detection (repere robot, y devant, x a droite) :
 * \verbatim
 *   0  0  0
 *   3  3  3     level 3 : zone moyenne (threshold_Front)
 *  2G  4  1D    level 4 : dead center, level 1/2 : cotes (threshold_veryclosed)
 *   3  3  3     level -3 : zone arriere moyenne
 *   0  0  0
 * \endverbatim
 *
 * Les niveaux positifs (1-4) = devant, negatifs (-1 a -4) = derriere, 0 = hors zone.
 */
class ObstacleZone {
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("ObstacleZone");
		return instance;
	}

	// --- Configuration capteurs ---
	bool enableFrontLeft_;
	bool enableFrontCenter_;
	bool enableFrontRight_;

	bool enableBackLeft_;
	bool enableBackCenter_;
	bool enableBackRight_;

	bool ignoreFrontLeft_;
	bool ignoreFrontCenter_;
	bool ignoreFrontRight_;

	bool ignoreBackLeft_;
	bool ignoreBackCenter_;
	bool ignoreBackRight_;

	// --- Seuils de distance (mm) ---
	int frontLeftThreshold_;
	int frontCenterThreshold_;
	int frontRightThreshold_;

	int frontLeftVeryClosedThreshold_;
	int frontCenterVeryClosedThreshold_;
	int frontRightVeryClosedThreshold_;

	int backLeftThreshold_;
	int backCenterThreshold_;
	int backRightThreshold_;

	int backLeftVeryClosedThreshold_;
	int backCenterVeryClosedThreshold_;
	int backRightVeryClosedThreshold_;

	// --- Flags de detection ---
	bool adv_is_detected_front_right_;
	bool adv_is_detected_front_left_;
	bool adv_is_detected_back_right_;
	bool adv_is_detected_back_left_;

	bool remove_outside_table_;

public:

	ObstacleZone()
	{
		enableFrontLeft_ = false;
		enableFrontCenter_ = false;
		enableFrontRight_ = false;
		enableBackLeft_ = false;
		enableBackCenter_ = false;
		enableBackRight_ = false;

		ignoreFrontLeft_ = false;
		ignoreFrontCenter_ = false;
		ignoreFrontRight_ = false;
		ignoreBackLeft_ = false;
		ignoreBackCenter_ = false;
		ignoreBackRight_ = false;

		frontLeftThreshold_ = 0;
		frontCenterThreshold_ = 0;
		frontRightThreshold_ = 0;
		frontLeftVeryClosedThreshold_ = 0;
		frontCenterVeryClosedThreshold_ = 0;
		frontRightVeryClosedThreshold_ = 0;

		backLeftThreshold_ = 0;
		backCenterThreshold_ = 0;
		backRightThreshold_ = 0;
		backLeftVeryClosedThreshold_ = 0;
		backCenterVeryClosedThreshold_ = 0;
		backRightVeryClosedThreshold_ = 0;

		adv_is_detected_front_right_ = false;
		adv_is_detected_front_left_ = false;
		adv_is_detected_back_right_ = false;
		adv_is_detected_back_left_ = false;

		remove_outside_table_ = true;
	}

	~ObstacleZone() {}

	// ========== Configuration capteurs ==========

	void addConfigFront(bool left, bool center, bool right)
	{
		enableFrontLeft_ = left;
		enableFrontCenter_ = center;
		enableFrontRight_ = right;
	}

	void addConfigBack(bool left, bool center, bool right)
	{
		enableBackLeft_ = left;
		enableBackCenter_ = center;
		enableBackRight_ = right;
	}

	// ========== Seuils ==========

	void addThresholdFront(int left, int center, int right)
	{
		frontLeftThreshold_ = left;
		frontCenterThreshold_ = center;
		frontRightThreshold_ = right;
	}

	void addThresholdFrontVeryClosed(int left, int center, int right)
	{
		frontLeftVeryClosedThreshold_ = left;
		frontCenterVeryClosedThreshold_ = center;
		frontRightVeryClosedThreshold_ = right;
	}

	void addThresholdBack(int left, int center, int right)
	{
		backLeftThreshold_ = left;
		backCenterThreshold_ = center;
		backRightThreshold_ = right;
	}

	void addThresholdBackVeryClosed(int left, int center, int right)
	{
		backLeftVeryClosedThreshold_ = left;
		backCenterVeryClosedThreshold_ = center;
		backRightVeryClosedThreshold_ = right;
	}

	// ========== Ignore flags ==========

	void setIgnoreFrontNearObstacle(bool ignoreLeft, bool ignoreCenter, bool ignoreRight)
	{
		ignoreFrontLeft_ = ignoreLeft;
		ignoreFrontCenter_ = ignoreCenter;
		ignoreFrontRight_ = ignoreRight;
	}

	void setIgnoreBackNearObstacle(bool ignoreLeft, bool ignoreCenter, bool ignoreRight)
	{
		ignoreBackLeft_ = ignoreLeft;
		ignoreBackCenter_ = ignoreCenter;
		ignoreBackRight_ = ignoreRight;
	}

	void setIgnoreAllFrontNearObstacle(bool ignore)
	{
		ignoreFrontLeft_ = ignore;
		ignoreFrontCenter_ = ignore;
		ignoreFrontRight_ = ignore;
	}

	void setIgnoreAllBackNearObstacle(bool ignore)
	{
		ignoreBackLeft_ = ignore;
		ignoreBackCenter_ = ignore;
		ignoreBackRight_ = ignore;
	}

	// ========== Disponibilite ==========

	inline bool getAvailableFrontCenter() const
	{
		return (enableFrontCenter_ & !ignoreFrontCenter_);
	}

	inline bool getAvailableBackCenter() const
	{
		return (enableBackCenter_ & !ignoreBackCenter_);
	}

	// ========== Detection flags ==========

	inline int right() const
	{
		return adv_is_detected_front_right_ | adv_is_detected_back_right_;
	}

	inline int left() const
	{
		return adv_is_detected_front_left_ | adv_is_detected_back_left_;
	}

	inline void setDetectedFrontRight(bool v) { adv_is_detected_front_right_ = v; }
	inline void setDetectedFrontLeft(bool v) { adv_is_detected_front_left_ = v; }
	inline void setDetectedBackRight(bool v) { adv_is_detected_back_right_ = v; }
	inline void setDetectedBackLeft(bool v) { adv_is_detected_back_left_ = v; }

	// ========== Filtrage table ==========

	inline void setRemoveOutsideTable(bool enable) { remove_outside_table_ = enable; }
	inline bool removeOutsideTable() const { return remove_outside_table_; }

	// ========== Accesseurs seuils (pour Sensors::front/back) ==========

	inline int frontLeftThreshold() const { return frontLeftThreshold_; }
	inline int frontCenterThreshold() const { return frontCenterThreshold_; }
	inline int frontRightThreshold() const { return frontRightThreshold_; }
	inline int frontLeftVeryClosedThreshold() const { return frontLeftVeryClosedThreshold_; }
	inline int frontCenterVeryClosedThreshold() const { return frontCenterVeryClosedThreshold_; }
	inline int frontRightVeryClosedThreshold() const { return frontRightVeryClosedThreshold_; }

	inline int backLeftThreshold() const { return backLeftThreshold_; }
	inline int backCenterThreshold() const { return backCenterThreshold_; }
	inline int backRightThreshold() const { return backRightThreshold_; }
	inline int backLeftVeryClosedThreshold() const { return backLeftVeryClosedThreshold_; }
	inline int backCenterVeryClosedThreshold() const { return backCenterVeryClosedThreshold_; }
	inline int backRightVeryClosedThreshold() const { return backRightVeryClosedThreshold_; }

	inline bool enableFrontLeft() const { return enableFrontLeft_; }
	inline bool enableFrontCenter() const { return enableFrontCenter_; }
	inline bool enableFrontRight() const { return enableFrontRight_; }
	inline bool enableBackLeft() const { return enableBackLeft_; }
	inline bool enableBackCenter() const { return enableBackCenter_; }
	inline bool enableBackRight() const { return enableBackRight_; }

	inline bool ignoreFrontLeft() const { return ignoreFrontLeft_; }
	inline bool ignoreFrontCenter() const { return ignoreFrontCenter_; }
	inline bool ignoreFrontRight() const { return ignoreFrontRight_; }
	inline bool ignoreBackLeft() const { return ignoreBackLeft_; }
	inline bool ignoreBackCenter() const { return ignoreBackCenter_; }
	inline bool ignoreBackRight() const { return ignoreBackRight_; }

	// ========== Filtres de classification ==========

	/*!
	 * \brief Classifie un obstacle devant le robot par zone.
	 *
	 * Repere robot : y devant, x a droite. Applique un patch balise +/-50mm.
	 *
	 * \param threshold_LR_mm Seuil lateral (separation centre/cote).
	 * \param threshold_Front_mm Seuil frontal maximum.
	 * \param threshold_veryclosed_front_mm Seuil tres proche (emergency).
	 * \param dist_adv_mm Distance adversaire (non utilise dans le calcul actuel).
	 * \param x_adv_mm Position X adversaire dans le repere robot (mm).
	 * \param y_adv_mm Position Y adversaire dans le repere robot (mm).
	 * \param theta_adv_deg Angle adversaire (non utilise dans le calcul actuel).
	 * \return Niveau 0 (hors zone), 1 (droite), 2 (gauche), 3 (moyen), 4 (dead center).
	 */
	int filtre_levelInFront(int threshold_LR_mm, int threshold_Front_mm,
			int threshold_veryclosed_front_mm,
			float dist_adv_mm, float x_adv_mm, float y_adv_mm,
			float theta_adv_deg);

	/*!
	 * \brief Classifie un obstacle derriere le robot par zone.
	 *
	 * Symetrique de filtre_levelInFront pour y < 0.
	 *
	 * \return Niveau 0 (hors zone), -1 (droite), -2 (gauche), -3 (moyen), -4 (dead center).
	 */
	int filtre_levelInBack(int threshold_LR_mm, int threshold_Back_mm,
			int threshold_veryclosed_back_mm,
			float dist_adv_mm, float x_adv_mm, float y_adv_mm,
			float theta_adv_deg);
};

#endif
