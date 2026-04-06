/*!
 * \file
 * \brief Implementation des filtres de classification d'obstacles.
 */

#include "ObstacleZone.hpp"

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

	if (ydist_adv > 0)
	{
		// return 1 si c'est à droite
		if ((ydist_adv <= threshold_veryclosed_front_mm)
				&& (xdist_adv >= threshold_LR_mm)
				&& (xdist_adv <= threshold_Front_mm))
		{
			return 1;
		}
		// return 2 si c'est à gauche
		if ((ydist_adv <= threshold_veryclosed_front_mm)
				&& (xdist_adv <= (-threshold_LR_mm))
				&& (xdist_adv >= -threshold_Front_mm))
		{
			return 2;
		}
		// return 3 zone moyenne
		if ((ydist_adv <= threshold_Front_mm)
				&& (ydist_adv > threshold_veryclosed_front_mm)
				&& (xdist_adv >= -threshold_Front_mm)
				&& (xdist_adv <= threshold_Front_mm))
		{
			return 3;
		}
		// return 4 dead center tres proche
		if ((ydist_adv <= threshold_veryclosed_front_mm)
				&& (xdist_adv >= -threshold_LR_mm)
				&& (xdist_adv <= threshold_LR_mm))
		{
			return 4;
		}
	}
	return 0;
}

int ObstacleZone::filtre_levelInBack(int threshold_LR_mm, int threshold_Back_mm,
		int threshold_veryclosed_back_mm,
		float dist_adv_mm, float x_adv_mm, float y_adv_mm,
		float theta_adv_deg)
{
	logger().error() << __FUNCTION__ << " threshold_LR_mm=" << threshold_LR_mm
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

	if (ydist_adv < 0)
	{
		// return -1 si c'est à droite
		if ((ydist_adv >= -threshold_veryclosed_back_mm)
				&& (xdist_adv >= threshold_LR_mm)
				&& (xdist_adv <= threshold_Back_mm))
		{
			return -1;
		}
		// return -2 si c'est à gauche
		if ((ydist_adv >= -threshold_veryclosed_back_mm)
				&& (xdist_adv <= (-threshold_LR_mm))
				&& (xdist_adv >= -threshold_Back_mm))
		{
			return -2;
		}
		// return -3 zone moyenne
		if ((ydist_adv >= -threshold_Back_mm)
				&& (ydist_adv < -threshold_veryclosed_back_mm)
				&& (xdist_adv >= -threshold_Back_mm)
				&& (xdist_adv <= threshold_Back_mm))
		{
			return -3;
		}
		// return -4 dead center tres proche
		if ((ydist_adv >= -threshold_veryclosed_back_mm)
				&& (xdist_adv >= -threshold_LR_mm)
				&& (xdist_adv <= threshold_LR_mm))
		{
			return -4;
		}
	}
	return 0;
}
