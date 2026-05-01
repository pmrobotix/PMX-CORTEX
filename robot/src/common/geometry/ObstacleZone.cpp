/*!
 * \file
 * \brief Implementation des filtres de classification d'obstacles.
 */

#include "ObstacleZone.hpp"

#include <cmath>

// Convention canonique repere robot (alignee balise Teensy + simu) :
//   x_adv_mm > 0  : adv DEVANT le robot
//   y_adv_mm > 0  : adv a GAUCHE  (sens trigo positif, 0 deg = devant)
//   y_adv_mm < 0  : adv a DROITE
int ObstacleZone::filtre_levelInFront(int threshold_LR_mm, int threshold_Front_mm,
		int threshold_veryclosed_front_mm,
		float dist_adv_mm, float x_adv_mm, float y_adv_mm,
		float theta_adv_deg)
{
	logger().debug() << __FUNCTION__ << " threshold_LR_mm=" << threshold_LR_mm
			<< " threshold_Front_mm=" << threshold_Front_mm
			<< " threshold_veryclosed_front_mm=" << threshold_veryclosed_front_mm
			<< " dist_mm=" << dist_adv_mm
			<< " x_mm=" << x_adv_mm << " y_mm=" << y_adv_mm
			<< " theta_adv_deg=" << theta_adv_deg << logs::end;

	int xdist_adv = (int) x_adv_mm;
	int ydist_adv = (int) y_adv_mm;

	//patch balise!!!!!!!!!!!!!!!!
	if (xdist_adv > 0) xdist_adv += 50;
	if (xdist_adv < 0) xdist_adv -= 50;
	if (ydist_adv > 0) ydist_adv += 50;
	if (ydist_adv < 0) ydist_adv -= 50;

	if (xdist_adv > 0)  // adv devant
	{
		// return 1 si c'est à droite (y < 0)
		if ((xdist_adv <= threshold_veryclosed_front_mm)
				&& (ydist_adv <= -threshold_LR_mm)
				&& (ydist_adv >= -threshold_Front_mm))
		{
			return 1;
		}
		// return 2 si c'est à gauche (y > 0)
		if ((xdist_adv <= threshold_veryclosed_front_mm)
				&& (ydist_adv >= threshold_LR_mm)
				&& (ydist_adv <= threshold_Front_mm))
		{
			return 2;
		}
		// return 3 zone moyenne (entre veryclosed et front, lateral dans [-front, +front])
		if ((xdist_adv <= threshold_Front_mm)
				&& (xdist_adv > threshold_veryclosed_front_mm)
				&& (ydist_adv >= -threshold_Front_mm)
				&& (ydist_adv <= threshold_Front_mm))
		{
			return 3;
		}
		// return 4 dead center tres proche
		if ((xdist_adv <= threshold_veryclosed_front_mm)
				&& (ydist_adv >= -threshold_LR_mm)
				&& (ydist_adv <= threshold_LR_mm))
		{
			return 4;
		}
	}
	return 0;
}

// Convention canonique : x < 0 = adv DERRIERE le robot, y = lateral (gauche>0).
int ObstacleZone::filtre_levelInBack(int threshold_LR_mm, int threshold_Back_mm,
		int threshold_veryclosed_back_mm,
		float dist_adv_mm, float x_adv_mm, float y_adv_mm,
		float theta_adv_deg)
{
	logger().debug() << __FUNCTION__ << " threshold_LR_mm=" << threshold_LR_mm
			<< " threshold_Back_mm=" << threshold_Back_mm
			<< " threshold_veryclosed_back_mm=" << threshold_veryclosed_back_mm
			<< " dist_mm=" << dist_adv_mm
			<< " x_adv_mm=" << x_adv_mm << " y_adv_mm=" << y_adv_mm
			<< " theta_adv_deg=" << theta_adv_deg << logs::end;

	int xdist_adv = (int) x_adv_mm;
	int ydist_adv = (int) y_adv_mm;

	//patch balise!!!!!!!!!!!!!!!!
	if (xdist_adv > 0) xdist_adv += 50;
	if (xdist_adv < 0) xdist_adv -= 50;
	if (ydist_adv > 0) ydist_adv += 50;
	if (ydist_adv < 0) ydist_adv -= 50;

	if (xdist_adv < 0)  // adv derriere
	{
		// return -1 si c'est à droite (y < 0)
		if ((xdist_adv >= -threshold_veryclosed_back_mm)
				&& (ydist_adv <= -threshold_LR_mm)
				&& (ydist_adv >= -threshold_Back_mm))
		{
			return -1;
		}
		// return -2 si c'est à gauche (y > 0)
		if ((xdist_adv >= -threshold_veryclosed_back_mm)
				&& (ydist_adv >= threshold_LR_mm)
				&& (ydist_adv <= threshold_Back_mm))
		{
			return -2;
		}
		// return -3 zone moyenne
		if ((xdist_adv >= -threshold_Back_mm)
				&& (xdist_adv < -threshold_veryclosed_back_mm)
				&& (ydist_adv >= -threshold_Back_mm)
				&& (ydist_adv <= threshold_Back_mm))
		{
			return -3;
		}
		// return -4 dead center tres proche
		if ((xdist_adv >= -threshold_veryclosed_back_mm)
				&& (ydist_adv >= -threshold_LR_mm)
				&& (ydist_adv <= threshold_LR_mm))
		{
			return -4;
		}
	}
	return 0;
}

// ========== Detection predictive par trajectoire ==========

PathStatus ObstacleZone::isOnPath(
		float x_robot, float y_robot,
		float x_target, float y_target,
		float x_adv, float y_adv,
		float corridor_width_mm,
		float slow_distance_mm,
		float stop_distance_mm) const
{
	const float dx = x_target - x_robot;
	const float dy = y_target - y_robot;
	const float len_sq = dx * dx + dy * dy;

	// Segment nul (rotation pure, robot == cible) : pas de trajectoire a bloquer.
	if (len_sq < 1.0f) {
		return PathStatus::CLEAR;
	}

	// Projection orthogonale du centre adv sur le segment.
	// t = 0 -> centre robot, t = 1 -> centre cible, t hors [0,1] -> hors segment.
	const float t = ((x_adv - x_robot) * dx + (y_adv - y_robot) * dy) / len_sq;
	if (t < 0.0f || t > 1.0f) {
		return PathStatus::CLEAR;
	}

	// Distance perpendiculaire centre_adv -> segment (comparee au carre pour eviter sqrt).
	const float px = x_robot + t * dx;
	const float py = y_robot + t * dy;
	const float lat_sq = (x_adv - px) * (x_adv - px) + (y_adv - py) * (y_adv - py);
	const float half_w = corridor_width_mm * 0.5f;

	// Bord du couloir exclusif : lat >= half_w -> hors couloir.
	if (lat_sq >= half_w * half_w) {
		return PathStatus::CLEAR;
	}

	// Dans le couloir : classification selon distance le long du segment.
	const float along = t * std::sqrt(len_sq);
	if (along <= stop_distance_mm) {
		return PathStatus::BLOCKING;
	}
	if (along <= slow_distance_mm) {
		return PathStatus::APPROACHING;
	}
	return PathStatus::CLEAR;
}
