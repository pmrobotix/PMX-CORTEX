/*!
 * \file OPOS6UL_ActionsExtended.hpp
 * \brief Extension des actions du robot pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_ACTIONSEXTENDED_HPP_
#define OPOS6UL_ACTIONSEXTENDED_HPP_

#include <string>
#include <iostream>

#include "action/Actions.hpp"
#include "action/ButtonBar.hpp"
#include "action/LcdShield.hpp"
#include "action/LedBar.hpp"
#include "action/Sensors.hpp"
#include "action/ServoObjectsSystem.hpp"
#include "action/Tirette.hpp"
#include "interface/AServoDriver.hpp"
#include "action/ServoUsingMotor.hpp"
#include "interface/AColorDriver.hpp"

/*!
 * \brief Gestion des actions étendues du robot OPOS6UL (servos, capteurs, LEDs, LCD).
 *
 * Fournit l'accès aux périphériques d'action et les commandes de servomoteurs AX12.
 *
 * == Lamp test et diagnostic hardware (LEDs) ==
 *
 * Au demarrage, les 8 LEDs s'allument (vert, 500ms).
 * Chaque LED s'eteint quand le check du composant correspondant passe OK.
 * Une LED qui reste allumee = composant en erreur.
 * Les logs sont prefixes "Hardware status:" pour grep.
 *
 *   LED 0 = LcdShield          (MCP23017 I2C)
 *   LED 1 = Tirette/Switch     (PCA9555 I2C)
 *   LED 2 = BeaconSensors      (Teensy I2C)
 *   LED 3 = GroveColorSensor   (TCS3414 I2C)
 *   LED 4 = Servos AX12        (Teensy CCAx12 I2C)
 *   LED 5 = (reserve)
 *   LED 6 = (reserve)
 *   LED 7 = AsservDriver       (Nucleo serie USB)
 *
 * Voir aussi ARCHITECTURE.md section "Hardware status LEDs".
 */
class OPOS6UL_ActionsExtended: public Actions {
private:

	static inline const logs::Logger& logger()
	{
		static const logs::Logger &instance = logs::LoggerFactory::logger("OPOS6UL_ActionsExtended");
		return instance;
	}

	/*!
	 * \brief LED Bar.
	 */
	LedBar ledbar_;

	/*!
	 * \brief Button Bar.
	 */
	ButtonBar buttonbar_;

	/*!
	 * \brief LcdShield 2x16 characters.
	 */
	LcdShield lcd2x16_;

	/*!
	 * \brief la tirette du robot.
	 */
	Tirette tirette_;

	/*!
	 * \brief capteurs IR/US.
	 */
	Sensors sensors_;

	/*!
	 * \brief objets avec servomotors ax12 et std.
	 */
	ServoObjectsSystem servos_;

	//ServoUsingMotor lanceurCerises_;

	/*!
	 * \brief capteur de couleur Grove (TCS3414 I2C).
	 */
	AColorDriver *colordriver_;

	/*!
	 * \brief Statut connexion AX12 Teensy. Set par start() au boot via les
	 *        ping de setup. Lu par ax12_init() / ax12_GO_banderole() pour
	 *        skipper les move bloquants quand les servos sont absents
	 *        (sinon timeout ~2s par mouvement, cumule a chaque setPos).
	 */
	bool servosAx12Connected_ = false;

public:
	/*!
	 * \brief Indique si les servos AX12 ont repondu au ping initial (start()).
	 */
	bool isServosAx12Connected() const { return servosAx12Connected_; }


	/*!
	 * \brief Enumération des libellés des servos associés au numéro de servo
	 * port-servo port1-4=1000-4000 servo=1-255 => 1000 à 4255
	 * [num port] * 1000 + [num servo]
	 */
	enum ServoAx12Label {
		AX12_SERVO_BRAS_D = 1 * 1000 + 7, //AX12
		AX12_SERVO_BRAS_G = 1000 + 5, //AX12

		AX12_enumTypeEnd
	};

	/*!
	 * \brief Enumération des libellés des servos STD associés au numéro de servo
	 *  port=10000
	 */
	enum ServoStdLabel {

		STD_SERVO_3 = 10000 * 5,

		STD_SERVO_4 = 10000 * 4,

		SERVO_enumTypeEnd
	};

	OPOS6UL_ActionsExtended(std::string botId, Robot *robot);

	~OPOS6UL_ActionsExtended()
	{
	}

	/*!
	 * \brief Cette methode retourne l'objet ledbar.
	 * \return ledbar_.
	 */
	LedBar& ledBar()
	{
		return ledbar_;
	}

	/*!
	 * \brief Retourne l'objet ButtonBar.
	 * \return Référence vers buttonbar_.
	 */
	ButtonBar& buttonBar()
	{
		return buttonbar_;
	}

	/*!
	 * \brief Cette methode retourne l'objet LcdShield.
	 * \return lcd2x16_.
	 */
	LcdShield& lcd2x16()
	{
		return lcd2x16_;
	}

	/*!
	 * \brief Cette methode retourne l'objet tirette.
	 * \return tirette_.
	 */
	Tirette& tirette()
	{
		return tirette_;
	}

	/*!
	 * \brief Cette methode retourne l'objet lanceur.
	 * \return tirette_.
	 */
//    ServoUsingMotor& lanceur()
//    {
//        return lanceurCerises_;
//    }
	/*!
	 * \brief Cette methode retourne l'objet sensors.
	 * \return sensors_.
	 */
	Sensors& sensors()
	{
		return sensors_;
	}

	/*!
	 * \brief Cette methode retourne l'objet servos.
	 * \return servos_.
	 */
	ServoObjectsSystem& servos()
	{
		return servos_;
	}

	/*!
	 * \brief Arrête tous les périphériques supplémentaires (capteurs, LEDs, LCD) et relâche les servos.
	 */
	void stopExtra()
	{

		sensors_.stopSensorsThread();
		ledbar_.stop();

		ledbar_.resetAll();
		lcd2x16_.reset();

		releaseAll();

	}

	//--------------------------------------------------------------
	//Actions 2026
	//--------------------------------------------------------------

	/*!
	 * \brief Relâche tous les servomoteurs AX12 (coupe le maintien en position).
	 */
	void releaseAll()
	{
		//logger().error() << "releaseAll()" << logs::end;
		servos().release(AX12_SERVO_BRAS_D);
		servos().release(AX12_SERVO_BRAS_G);
	

	}

	/*!
	 * \brief Initialise tous les servomoteurs AX12 à leur position de départ.
	 */
	void ax12_init()
	{

		// Si les AX12 n'ont pas pinge au boot, skip les move bloquants
		// (chaque ax12_*_banderole/R/L est un move 2s qui timeout sans servo).
		// Permet de gagner ~2s par appel a setPos() en config sans servos.
		if (!servosAx12Connected_) {
			logger().warn() << "ax12_init() skip : servos AX12 non connectes" << logs::end;
			return;
		}


		ax12_bras_droit();
		ax12_bras_gauche(-1);
		ax12_bras_droit_init();
		ax12_bras_gauche_init(-1);
		

	}

	/*!
	 * \brief Place le bras droit en position initiale (replié).
	 * \param keep Durée de maintien en ms (0 = maintien continu).
	 * \param eta Durée estimée du mouvement en ms.
	 */
	void ax12_bras_droit_init(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_D, speed);
		//servos().deploy(AX12_SERVO_BRAS_D, 815, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_D, 215, keep);
	}

	/*!
	 * \brief Déploie le bras droit en position d'action.
	 * \param keep Durée de maintien en ms (0 = maintien continu).
	 * \param eta Durée estimée du mouvement en ms.
	 */
	void ax12_bras_droit(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_D, speed);
		//servos().deploy(AX12_SERVO_BRAS_D, 480, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_D, 512, keep);
	}

	void ax12_bras_droit_half(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_D, speed);
		//servos().deploy(AX12_SERVO_BRAS_D, 815, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_D, 370, keep);
	}

//	void ax12_bras_droit_full(int keep = 0, int eta = 400)
//	{
//		servos().move_1_servo(eta, AX12_SERVO_BRAS_D, 812, keep);
//	}

	/*!
	 * \brief Place le bras gauche en position initiale (replié).
	 * \param keep Durée de maintien en ms (0 = maintien continu).
	 * \param eta Durée estimée du mouvement en ms.
	 */
	void ax12_bras_gauche_init(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_G, speed);
		//servos().deploy(AX12_SERVO_BRAS_G, 205, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_G, 816, keep);
	}

	/*!
	 * \brief Déploie le bras gauche en position d'action.
	 * \param keep Durée de maintien en ms (0 = maintien continu).
	 * \param eta Durée estimée du mouvement en ms.
	 */
	void ax12_bras_gauche(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_G, speed);
		//servos().deploy(AX12_SERVO_BRAS_G, 512, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_G, 512, keep);
	}

	void ax12_bras_gauche_half(int keep = 0, int eta = 400)
	{
		//servos().setSpeed(AX12_SERVO_BRAS_G, speed);
		//servos().deploy(AX12_SERVO_BRAS_G, 205, keep);
		servos().move_1_servo(eta, AX12_SERVO_BRAS_G, 700, keep);
	}

//	void ax12_bras_gauche_full(int keep = 0, int eta = 400)
//	{
//
//		servos().move_1_servo(eta, AX12_SERVO_BRAS_G, 222, keep);
//	}
};

#endif
