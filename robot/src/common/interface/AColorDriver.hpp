/*!
 * \file
 * \brief Interface abstraite du driver de capteur couleur.
 *
 * Gere un capteur couleur (Grove Color Sensor) pour la detection
 * de couleurs sur les elements de jeu.
 */

#ifndef ACOLORDRIVER_HPP_
#define ACOLORDRIVER_HPP_

#include <string>

/*!
 * \brief Interface abstraite du driver de capteur couleur.
 *
 * Fournit la lecture des coordonnees chromatiques (Tx, Ty)
 * depuis un capteur couleur I2C. L'implementation ARM lit le capteur
 * reel, il n'y a pas d'implementation SIMU (ARM seulement).
 */
class AColorDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \param botName Nom du robot.
	 * \return Pointeur vers l'instance creee.
	 */
	static AColorDriver * create(std::string botName);

	virtual ~AColorDriver()
	{
	}

	/*!
	 * \brief Verifie si le capteur couleur est connecte.
	 * \return true si la communication I2C est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Effectue une lecture RGB du capteur.
	 * \return true si la lecture a reussi (capteur connecte).
	 */
	virtual bool readRGB() = 0;

	/*!
	 * \brief Retourne la coordonnee chromatique Tx.
	 * \return Valeur Tx (espace CIE xy).
	 */
	virtual float getTX() = 0;

	/*!
	 * \brief Retourne la coordonnee chromatique Ty.
	 * \return Valeur Ty (espace CIE xy).
	 */
	virtual float getTY() = 0;

protected:

	AColorDriver()
	{
	}

};

#endif
