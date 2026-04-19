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
 * \brief Statut d'un adversaire vis-a-vis d'une trajectoire.
 *
 * Utilise par ObstacleZone::isOnPath pour classifier un adversaire
 * par rapport au segment [robot -> cible] du mouvement courant.
 */
enum class PathStatus
{
	CLEAR,       //!< Adversaire hors du couloir ou hors segment : la trajectoire est libre.
	APPROACHING, //!< Adversaire dans le couloir, distance entre stop et slow : ralentir.
	BLOCKING,    //!< Adversaire dans le couloir, distance <= stop : arreter.
};

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

	// ========== Detection predictive par trajectoire ==========

	/*!
	 * \brief Classifie un adversaire par rapport au segment [robot -> cible].
	 *
	 * Geometrie pure, sans effet de bord. Toutes les positions sont des CENTRES
	 * (repere table), pas des bords de robot.
	 *
	 * Deux distances sont calculees et comparees a des seuils :
	 *
	 * 1) Distance PERPENDICULAIRE (lateral_mm) : du centre adv au segment.
	 *    Compare a corridor_width_mm / 2.
	 *      lateral_mm <= corridor_width_mm / 2  : adv DANS le couloir
	 *      lateral_mm >  corridor_width_mm / 2  : adv HORS couloir -> CLEAR
	 *
	 *    corridor_width_mm est centre-a-centre :
	 *      corridor_width_mm = robot_diametre + adv_diametre (avec marge incluse
	 *      cote appelant en augmentant adv_diametre).
	 *
	 * 2) Distance LONGITUDINALE (along_mm) : du centre robot a la projection
	 *    orthogonale du centre adv sur le segment, mesuree LE LONG du segment.
	 *      along_mm <  0                   : projection derriere le robot -> CLEAR
	 *      along_mm >  longueur_segment    : projection au-dela de la cible -> CLEAR
	 *      along_mm <= stop_distance_mm    : BLOCKING (stop) [inclusif]
	 *      along_mm <= slow_distance_mm    : APPROACHING (slow) [inclusif]
	 *      along_mm >  slow_distance_mm    : CLEAR
	 *
	 * Illustration :
	 *
	 *   corridor_width/2       cible
	 *         ▲                  │
	 *         │          lateral │ (distance perpendiculaire adv -> segment)
	 *         ▼         ├────────●
	 *   ┌─────────┬─────┬────────┴─┐      ← limite couloir (corridor_width/2)
	 *   │         │     │          │
	 *   │ robot   │     │  adv     │      ← adv dans le couloir si lateral <= width/2
	 *   │  ●──────┼─────┼──────────┼──▶ cible
	 *   │  0     stop  slow       end
	 *   │         │     │          │      ← limite couloir
	 *   └─────────┴─────┴──────────┘
	 *   ├────────▶│                       along_mm (distance LE LONG du segment,
	 *             │                        depuis centre robot jusqu'a la
	 *                                      projection du centre adv)
	 *
	 *   Note : stop_distance_mm et slow_distance_mm sont mesures A PARTIR DU
	 *   CENTRE DU ROBOT (along_mm = 0 au centre robot, = longueur_segment a la cible).
	 *
	 * Cas particulier : segment nul (robot == cible, rotation pure) -> CLEAR toujours.
	 *
	 * \param x_robot,y_robot Centre robot en mm (repere table).
	 * \param x_target,y_target Centre cible en mm (repere table).
	 * \param x_adv,y_adv Centre adversaire en mm (repere table).
	 * \param corridor_width_mm Largeur totale du couloir centre-a-centre, en mm
	 *        (robot_diametre + adv_diametre_avec_marge). La demi-largeur
	 *        corridor_width_mm/2 est le seuil max de la distance perpendiculaire.
	 * \param slow_distance_mm Distance le long du segment depuis le CENTRE ROBOT,
	 *        seuil APPROACHING. Ex. 620 mm.
	 * \param stop_distance_mm Distance le long du segment depuis le CENTRE ROBOT,
	 *        seuil BLOCKING (inclusif). Ex. 460 mm.
	 * \return CLEAR / APPROACHING / BLOCKING.
	 */
	PathStatus isOnPath(
			float x_robot, float y_robot,
			float x_target, float y_target,
			float x_adv, float y_adv,
			float corridor_width_mm,
			float slow_distance_mm,
			float stop_distance_mm) const;
};

#endif
