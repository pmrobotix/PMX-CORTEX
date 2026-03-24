/*!
 * \file
 * \brief Interface abstraite du driver de moteur utilise en mode servomoteur.
 *
 * Gere un moteur DC avec encodeur utilise comme un servomoteur
 * (positionnement par asservissement interne).
 */

#ifndef ASERVOUSINGMOTORDRIVER_HPP_
#define ASERVOUSINGMOTORDRIVER_HPP_

/*!
 * \brief Interface abstraite du driver de moteur en mode servo.
 *
 * Fournit le controle d'un moteur DC utilise en positionnement :
 * commande de position avec rampe, lecture encodeur, arret.
 */
class AServoUsingMotorDriver
{

public:

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \return Pointeur vers l'instance creee.
	 */
	static AServoUsingMotorDriver* create();

	virtual ~AServoUsingMotorDriver()
	{
	}

	AServoUsingMotorDriver()
	{
	}

	/*!
	 * \brief Verifie si le moteur est connecte.
	 * \return true si la communication est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Commande le moteur vers une position cible.
	 * \param power Puissance appliquee (0-100).
	 * \param pos Position cible en ticks encodeur.
	 * \param ramptimems Duree de la rampe d'acceleration en ms (0 = instantane).
	 */
	virtual void setMotorPosition(int power, int pos, int ramptimems = 0) = 0;

	/*!
	 * \brief Lit la valeur de l'encodeur interne.
	 * \return Position en ticks.
	 */
	virtual long getInternalEncoder() = 0;

	/*!
	 * \brief Reinitialise l'encodeur a une valeur donnee.
	 * \param pos Nouvelle valeur de l'encodeur.
	 */
	virtual void resetEncoder(int pos) = 0;

	/*!
	 * \brief Arrete le moteur.
	 */
	virtual void stopMotor() = 0;

	/*!
	 * \brief Lit le courant consomme par le moteur.
	 * \return Courant en mA.
	 */
	virtual int getMotorCurrent() = 0;

};

#endif
