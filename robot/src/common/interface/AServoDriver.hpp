/*!
 * \file
 * \brief Interface abstraite du driver de servomoteurs.
 *
 * Gere les servomoteurs standards (PWM) et Dynamixel (AX-12).
 * Commandes de position, vitesse, couple, et etat.
 */

#ifndef ASERVODRIVER_HPP_
#define ASERVODRIVER_HPP_

/*!
 * \brief Interface abstraite du driver de servomoteurs.
 *
 * Fournit le controle des servomoteurs : positionnement par impulsion PWM,
 * mode moteur continu, gestion du couple. L'implementation ARM pilote
 * les servos reels (Dynamixel AX-12, PWM Adafruit), la SIMU est un stub.
 */
class AServoDriver
{

public:

	/*!
	 * \brief Type de servomoteur.
	 */
	enum ServoType
	{
		SERVO_STANDARD,  ///< Servo PWM classique.
		SERVO_DYNAMIXEL  ///< Servo Dynamixel (protocole serie).
	};

	/*!
	 * \brief Cree l'instance concrete selon la plateforme.
	 * \return Pointeur vers l'instance creee.
	 */
	static AServoDriver* create();

	virtual ~AServoDriver()
	{
	}

	AServoDriver()
	{
	}

	/*!
	 * \brief Limite une valeur a un intervalle [min, max].
	 * \param value Valeur a limiter.
	 * \param valeurMin Borne inferieure.
	 * \param valeurMax Borne superieure.
	 * \return Valeur limitee.
	 */
	long constrain(long value, long valeurMin, long valeurMax)
	{
		if (value < valeurMin)
			return valeurMin;
		if (value > valeurMax)
			return valeurMax;
		return value;
	}

	/*!
	 * \brief Verifie si la carte servo est connectee.
	 * \return true si la communication est etablie.
	 */
	virtual bool is_connected() = 0;

	/*!
	 * \brief Maintient la position courante du servo (active le couple).
	 * \param servo Index du servo.
	 */
	virtual void hold(int servo) = 0;

	/*!
	 * \brief Relache le servo (desactive le couple).
	 * \param servo Index du servo.
	 */
	virtual void release(int servo) = 0;

	/*!
	 * \brief Fait tourner le servo en mode moteur continu.
	 * \param servo Index du servo.
	 * \param speed Vitesse de rotation (signe = sens).
	 */
	virtual void turn(int servo, int speed) = 0;

	/*!
	 * \brief Definit la position du servo par largeur d'impulsion PWM.
	 * \param servo Index du servo.
	 * \param pulse Largeur d'impulsion en microsecondes.
	 * \param velocity Vitesse de deplacement (0 = max).
	 */
	virtual void setPulsePos(int servo, int pulse, int velocity = 0) = 0;

	/*!
	 * \brief Definit le couple maximal du servo.
	 * \param servo Index du servo.
	 * \param torque Valeur de couple.
	 */
	virtual void setTorque(int servo, int torque) = 0;

	/*!
	 * \brief Definit la largeur d'impulsion minimale.
	 * \param servo Index du servo.
	 * \param value Impulsion min en microsecondes.
	 */
	virtual void setMinPulse(int servo, int value) = 0;

	/*!
	 * \brief Definit la largeur d'impulsion centrale (position neutre).
	 * \param servo Index du servo.
	 * \param value Impulsion centrale en microsecondes.
	 */
	virtual void setMidPulse(int servo, int value) = 0;

	/*!
	 * \brief Definit la largeur d'impulsion maximale.
	 * \param servo Index du servo.
	 * \param value Impulsion max en microsecondes.
	 */
	virtual void setMaxPulse(int servo, int value) = 0;

	/*!
	 * \brief Inverse la polarite du servo.
	 * \param servo Index du servo.
	 * \param inversed true pour inverser.
	 */
	virtual void setPolarity(int servo, bool inversed) = 0;

	/*!
	 * \brief Definit le type de servo.
	 * \param servo Index du servo.
	 * \param type Type (SERVO_STANDARD ou SERVO_DYNAMIXEL).
	 * \deprecated A terme, le type sera fixe a la construction.
	 */
	virtual void setType(int servo, ServoType type) = 0;

	/*!
	 * \brief Envoie un ping au servo pour verifier sa presence.
	 * \param servo Index du servo.
	 * \return ID du servo si present, -1 sinon.
	 */
	virtual int ping(int servo) = 0;

	/*!
	 * \brief Verifie si le servo est en mouvement.
	 * \param servo Index du servo.
	 * \return 1 si en cours, 0 si termine, -1 si erreur.
	 */
	virtual int getMoving(int servo) = 0;

	/*!
	 * \brief Lit la position courante du servo.
	 * \param servo Index du servo.
	 * \return Largeur d'impulsion courante en microsecondes.
	 */
	virtual int getPulsePos(int servo) = 0;

	/*!
	 * \brief Lit le couple courant du servo.
	 * \param servo Index du servo.
	 * \return Valeur de couple.
	 */
	virtual int getTorque(int servo) = 0;

};

#endif
