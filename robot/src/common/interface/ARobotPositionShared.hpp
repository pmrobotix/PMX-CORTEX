/*!
 * \file
 * \brief Position partagee du robot et utilitaires geometriques.
 *
 * Definit la structure ROBOTPOSITION et l'interface abstraite
 * d'acces concurrent a la position du robot. Utilisee par
 * l'asservissement, les capteurs et la strategie.
 */

#ifndef _AROBOTPOSSHARED_HPP_
#define _AROBOTPOSSHARED_HPP_

#include <cmath>

#include "utils/Chronometer.hpp"

/*!
 * \brief Direction de deplacement du robot.
 */
enum MOVEMENT_DIRECTION {
	NONE,       ///< Aucun deplacement.
	FORWARD,    ///< Marche avant.
	BACKWARD,   ///< Marche arriere.
	TURN        ///< Rotation sur place.
};

/*!
 * \brief Position et etat du robot sur la table.
 */
struct sRobotPosition {
	float x;                ///< Position X en millimetres.
	float y;                ///< Position Y en millimetres.
	float theta;            ///< Angle en radians.

	/// Etat de l'asservissement : 0=idle, 1=running, 2=emergency stop, 3=blocked.
	int asservStatus;

	unsigned int queueSize; ///< Nombre de commandes en file d'attente.
	unsigned int debug_nb;  ///< Compteur de debug.
};
typedef struct sRobotPosition ROBOTPOSITION;

/*!
 * \brief Convertit des degres en radians.
 * \param deg Angle en degres.
 * \return Angle en radians.
 */
inline float degToRad(float deg)
{
	return deg * M_PI / 180.0;
}

/*!
 * \brief Convertit des radians en degres.
 * \param rad Angle en radians.
 * \return Angle en degres.
 */
inline float radToDeg(float rad)
{
	return rad * 180.0 / M_PI;
}

/*!
 * \brief Compare deux flottants avec une tolerance.
 * \param A Premier flottant.
 * \param B Second flottant.
 * \param epsilon Tolerance (defaut 0.005).
 * \return true si |A - B| < epsilon.
 */
inline bool cmpf(float A, float B, float epsilon = 0.005f)
{
	return (fabs(A - B) < epsilon);
}

/*!
 * \brief Normalise un angle dans l'intervalle ]-PI, PI].
 * \param rad Angle en radians.
 * \return Angle normalise en radians.
 */
inline float WrapAngle2PI(float rad)
{
	rad = std::fmod(rad, 2.0 * M_PI);
	if (rad < -M_PI) rad += (2.0 * M_PI);
	if (rad > M_PI) rad -= (2.0 * M_PI);
	return rad;
}

/*!
 * \brief Interface abstraite d'acces a la position partagee du robot.
 *
 * Fournit un acces thread-safe a la position du robot sur la table.
 * Utilisee par l'asservissement (ecriture) et par les capteurs/strategie (lecture).
 */
class ARobotPositionShared {

public:

	utils::Chronometer chrono_;  ///< Chronometre pour mesurer les temps entre mises a jour.

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \return Pointeur vers l'instance creee.
	 */
	static ARobotPositionShared* create();

	/*!
	 * \brief Lit la position courante du robot.
	 * \param debug Niveau de debug (0 = pas de log).
	 * \return Position et etat du robot.
	 */
	virtual ROBOTPOSITION getRobotPosition(int debug = 0) = 0;

	/*!
	 * \brief Met a jour la position du robot.
	 * \param p Nouvelle position.
	 */
	virtual void setRobotPosition(ROBOTPOSITION p) = 0;

	/*!
	 * \brief Convertit une position balise (polaire relative) en coordonnees table.
	 *
	 * Utilise la position courante du robot et les donnees de la balise ToF
	 * pour calculer la position absolue de l'objet detecte sur la table.
	 *
	 * \param d_mm Distance mesuree par la balise en mm.
	 * \param x_mm Coordonnee X brute de la balise (non utilisee dans le calcul).
	 * \param y_mm Coordonnee Y brute de la balise (non utilisee dans le calcul).
	 * \param theta_deg Angle de detection en degres (relatif a l'avant du robot).
	 * \param[out] x_botpos Coordonnee X calculee sur la table en mm.
	 * \param[out] y_botpos Coordonnee Y calculee sur la table en mm.
	 * \return Position du robot au moment de la conversion.
	 */
	ROBOTPOSITION convertPositionBeaconToRepereTable(float d_mm, float x_mm, float y_mm, float theta_deg,
			float *x_botpos, float *y_botpos)
	{
		ROBOTPOSITION p = getRobotPosition(0);

		float a = (p.theta - M_PI_2 + (theta_deg * M_PI / 180.0f));
		a = WrapAngle2PI(a);

		*x_botpos = p.x + (d_mm * cos(a));
		*y_botpos = p.y + (d_mm * sin(a));

		return p;
	}

	virtual ~ARobotPositionShared()
	{
	}

protected:

	ARobotPositionShared() :
			chrono_("ARobotPositionShared")
	{
	}

};

#endif
