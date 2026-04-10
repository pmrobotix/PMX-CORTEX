/*! \file ServoObjectsSystem.hpp
 * \brief Gestion du système de servomoteurs et timers de déplacement associés.
 */
#ifndef SERVOOBJECTSSYSTEM_HPP_
#define SERVOOBJECTSSYSTEM_HPP_

#include <sys/types.h>
#include <string>

#include "log/LoggerFactory.hpp"
#include "interface/AServoDriver.hpp"
#include "utils/Chronometer.hpp"
#include "AActionsElement.hpp"
#include "timer/ITimerScheduledListener.hpp"

#define TIMER_SERVO_PERIOD_US 50000
/*!
 * \brief Enumération des libellés des actions.
 */
enum ServoTimerName {
	MOVE_1_SERVO, MOVE_2_SERVOS
};

/*!
 * \brief Système de gestion des servomoteurs (déploiement, positionnement, torque).
 */
class ServoObjectsSystem: public AActionsElement, utils::Mutex {

private:
	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref ServoObjectsSystem.
	 */
	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("ServoObjectsSystem");
		return instance;
	}

	/*!
	 * \brief Pointeur vers le driver de servomoteurs.
	 */
	AServoDriver *servodriver_;

	/*!
	 * \brief ID du robot.
	 */
	std::string botId_;

	/*!
	 * \brief Indique si le mouvement en cours est terminé.
	 */
	bool move_finished_;

	//long move_starttime_ms_; //heure du start du move

	//Limitation d'une valeur à un intervalle [valeurMin , valeurMax]
	long constrain(long value, long valeurMin, long valeurMax)
	{
		if (value < valeurMin) return valeurMin;

		if (value > valeurMax) return valeurMax;

		return value;
	}

protected:

public:

	/*!
	 * \brief Constructor.
	 */
	ServoObjectsSystem(std::string botId, Actions &actions);

	/*!
	 * \brief Destructor.
	 */
	~ServoObjectsSystem();

	/*!
	 * \brief Retourne le pointeur vers le driver de servomoteurs.
	 */
	AServoDriver* servodriver()
	{
		return servodriver_;
	}

	/*!
	 * \brief Vérifie si le driver de servomoteurs est connecté.
	 */
	bool is_connected();
	/*!
	 * \brief setup 1 servo with type, min, mid, max, inv values.
	 */
	bool setup(int servo, AServoDriver::ServoType type, int valueMinPulse, int valueMidPulse, int valueMaxPulse,
			bool inversed = false);

	/*!
	 * \brief Arrête tous les timers de servomoteurs en cours.
	 */
	void stopTimers();

	/*!
	 * \brief move 1 servo.
	 */
	void move_1_servo(int time_eta_ms, int servo1, int pos1, int keep_torque_extratimems=0, int torque1=512,
			int escape_torque=1023, int mode=0);
	/*!
	 * \brief move 2 servos.
	 */
	void move_2_servos(int time_eta_ms,
			int servo1, int pos1,
			int servo2,	int pos2,
			int keep_torque_extratimems=0,
			int torque1=512, int torque2=512, int escape_torque=0, int mode=0);

	/*!
	 * \brief Déplace 3 servos simultanément.
	 */
	void move_3_servos(int time_eta_ms,
			int servo1, int pos1,
			int servo2,	int pos2,
			int servo3,	int pos3,
			int keep_torque_extratimems=0,
			int torque1=512, int torque2=512,int torque3=512,
			int escape_torque=0, int mode=0);
	/*!
	 * \brief Déplace 4 servos simultanément.
	 */
	void move_4_servos(int time_eta_ms,
				int servo1, int pos1,
				int servo2,	int pos2,
				int servo3,	int pos3,
				int servo4,	int pos4,
				int keep_torque_extratimems=0,
				int torque1=512, int torque2=512, int torque3=512,int torque4=512,
				 int escape_torque=0, int mode=0);

	//void deployByTimerTask(int servo, int pos, int keep_millisec = -1);
	/*!
	 * \brief Déploie un servo à la position donnée.
	 * \param keep_millisec Durée de maintien en ms (-1 = indéfini).
	 */
	void deploy(int servo, int pos, int keep_millisec = -1);

	/*!
	 * \brief Déploie un servo avec une vitesse contrôlée.
	 * \param velocity Vitesse (AX12 : 0-1023 ; servo std : temps en ms pour 0-90°).
	 * \param keep_millisec Durée de maintien en ms (-1 = indéfini).
	 */
	void deployWithVelocity(int servo, int pos, int velocity = 0, int keep_millisec = -1);

	/*!
	 * \brief Fait tourner un servo en continu à la vitesse donnée.
	 * \param keep_millisec Durée de rotation en ms (0 = indéfini).
	 */
	void turn(int servo, int speed, int keep_millisec = 0);

	/*!
	 * \brief Relâche le couple d'un servo (désactive le maintien).
	 */
	void release(int servo);

	/*!
	 * \brief Maintient le couple d'un servo à sa position actuelle.
	 */
	void hold(int servo);

	/*!
	 * \brief Retourne la valeur de couple actuelle d'un servo.
	 */
	int getTorque(int servo);

	/*!
	 * \brief Définit la valeur de couple d'un servo.
	 */
	void setTorque(int servo, int torque);

	/*!
	 * \brief Définit la vitesse d'un servo.
	 */
	void setSpeed(int servo, int speed);

	/*!
	 * \brief Retourne la vitesse actuelle d'un servo.
	 */
	int getSpeed(int servo);

	/*!
	 * \brief fonction avec vitesse reglable
	 * pos : AX12 : 0 - 512 - 1023
	 */
	int setPos(int servo, int pos, int milli0to90 = 0);

	/*!
	 * \brief get the pulsewidth
	 */
	int getPulseWidth(int servo);
	/*!
	 * \brief Détecte tous les servos connectés au bus.
	 */
	void detectAll();

	/*!
	 * \brief Détecte un servo spécifique.
	 */
	void detect();

	/*!
	 * \brief Retourne l'identifiant du robot.
	 */
	std::string id()
	{
		return botId_;
	}

	/*!
	 * \brief Retourne vrai si le mouvement en cours est terminé (thread-safe).
	 */
	bool move_finished()
	{
		bool b;
		lock();
		b = move_finished_;
		unlock();
		return b;
	}

	/*!
	 * \brief Définit l'état de fin de mouvement (thread-safe).
	 */
	void move_finished(bool finished)
	{
		lock();
		move_finished_ = finished;
		unlock();
	}
};

/*!
 * \brief Cette action permet de definir les timers concernant les servomotors.
 *
 */
class ServoObjectsTimer: public ITimerScheduledListener
{
private:

	/*!
	 * \brief Retourne le \ref Logger associé à la classe \ref ServoObjectsTimer.
	 */
	static const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("ServoObjectsTimer");
		return instance;
	}

	/*!
	 * \brief Référence vers le ledbar.
	 */
	ServoObjectsSystem &servoObjectsSystem_;

	int name_;
	int eta_ms_;
	//servo1
	int servo1_;
	int cur_pos1_;
	int goal_pos1_;
	int velocity1_;

	//servo2
	int servo2_;
	int cur_pos2_;
	int goal_pos2_;
	int velocity2_;

	long move_starttime_ms_; //heure du start du move
	bool first_exe_;

public:

	/*!
	 * \brief Constructeur de la classe.
	 * \param sensors
	 *        Reference vers l'objet associée.
	 */
	ServoObjectsTimer(ServoObjectsSystem &sOsS, int number_servos, uint timeSpan_us, int eta_ms, int servo1,
			int cur_pos1, int goal_pos1, int velocity);

	ServoObjectsTimer(ServoObjectsSystem &sOsS, int number_servos, uint timeSpan_us, int eta_ms, int servo1,
			int cur_pos1, int goal_pos1, int velocity1, int servo2, int cur_pos2, int goal_pos2, int velocity2);

	/*!
	 * \brief Destructeur de la classe.
	 */
	virtual inline ~ServoObjectsTimer()
	{
	}

	/*!
	 * \brief fonction qui sera executer à chaque traitement du timer.
	 */
	virtual void onTimer(utils::Chronometer chrono);

	/*!
	 * \brief fonction qui sera executer en dernière action à faire pour ce timer.
	 */
	virtual void onTimerEnd(utils::Chronometer chrono);

	/*!
	 * \brief nom du timer.
	 */
	virtual std::string info();

};

#endif
