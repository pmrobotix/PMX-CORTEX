/*!
 * \file
 * \brief Implémentation de la classe OPOS6UL_ActionsExtended.
 */

#include "OPOS6UL_ActionsExtended.hpp"
#include "Robot.hpp"
#include "asserv/Asserv.hpp"
#include "thread/Thread.hpp"

OPOS6UL_ActionsExtended::OPOS6UL_ActionsExtended(std::string botId, Robot *robot) :
		ledbar_(botId, *this, 8), buttonbar_(*this), lcd2x16_(botId, *this), tirette_(*this), sensors_(*this, robot), servos_(
				botId, *this) //,
//lanceurCerises_(botId, *this)

{
	// GroveColorSensor (pas dans la liste d'init, c'est un pointeur)
	colordriver_ = AColorDriver::create(botId);

	// LAMP TEST : allumer toutes les LEDs pour prouver que la barre fonctionne
	ledbar_.flashAll(LED_GREEN);
	utils::sleep_for_micros(500000); // 500ms visible

	// VERIF HARDWARE STATUS : chaque LED s'eteint si le driver est OK
	// LED 0 = LcdShield (MCP23017 I2C)
	if (!lcd2x16_.is_connected())
	{
		logger().error() << "Hardware status: LcdShield is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(0, LED_OFF);
		lcd2x16_.init();
		logger().debug() << "Hardware status: LcdShield OK" << logs::end;
	}

	// LED 1 = Tirette/Switch (PCA9555 I2C)
	if (!tirette_.is_connected())
	{
		logger().error() << "Hardware status: Tirette is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(1, LED_OFF);
		logger().debug() << "Hardware status: Tirette OK" << logs::end;
	}

	// LED 2 = BeaconSensors (Teensy I2C)
	if (!sensors_.is_connected())
	{
		logger().error() << "Hardware status: BeaconSensors is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(2, LED_OFF);
		logger().debug() << "Hardware status: BeaconSensors OK" << logs::end;
	}

	// LED 3 = GroveColorSensor (TCS3414 I2C)
	if (!colordriver_->is_connected())
	{
		logger().error() << "Hardware status: GroveColorSensor is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(3, LED_OFF);
		logger().debug() << "Hardware status: GroveColorSensor OK" << logs::end;
	}

	// LED 4 = Servos AX12 (Teensy CCAx12 I2C)
	int svrconnected = 0;
	svrconnected = servos().setup(AX12_SERVO_BRAS_D, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
	svrconnected &= servos().setup(AX12_SERVO_BRAS_G, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
	if (svrconnected == false)
	{
		logger().error() << "Hardware status: Servos AX12 is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(4, LED_OFF);
		logger().debug() << "Hardware status: Servos AX12 OK" << logs::end;
	}

	// LED 5-6 = (reserve)
	ledbar_.set(5, LED_OFF);
	ledbar_.set(6, LED_OFF);

	// LED 7 = AsservDriver (Nucleo serie USB)
	if (!robot->asserv().is_connected())
	{
		logger().error() << "Hardware status: AsservDriver is NOT connected !" << logs::end;
	}
	else
	{
		ledbar_.set(7, LED_OFF);
		logger().debug() << "Hardware status: AsservDriver OK" << logs::end;
	}
	/*
	 //config AX12 2023
	 int svrconnected = 0;
	 svrconnected = servos().setup(AX12_SERVO_BRAS_D, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
	 svrconnected &= servos().setup(AX12_SERVO_BRAS_G, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
	 if (svrconnected == false) {
	 ledbar_.set(4, LED_GREEN);
	 }

	 svrconnected = servos().setup(AX12_SERVO_ASPIRATION, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023,
	 false);
	 if (svrconnected == false) {
	 ledbar_.set(5, LED_GREEN);
	 }

	 svrconnected = servos().setup(AX12_SERVO_FUNNY, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
	 if (svrconnected == false) {
	 ledbar_.set(6, LED_GREEN);
	 }
	 */

//    svrconnected = servos().setup(AX12_SERVO_TETE_ASPI, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023, false);
//    svrconnected &= servos().setup(AX18_SERVO_RUSSEL_LINKAGE, AServoDriver::ServoType::SERVO_DYNAMIXEL, 0, 512, 1023,
//            false);
//    if (svrconnected == false) {
//        ledbar_.set(7, LED_GREEN);
//    }

	//config des sensors
	//TODO ajouter la configuration de la position des capteurs sensors_.addConfigPosFront(-140, 0, +140);
	sensors_.addConfigFront(false, true, false);
	sensors_.addConfigBack(false, true, false);

	int rayon_adv = 200;
	//int rayon_adv = 160;
	//int rayon_adv = 300;
	//rayon robot + espace elements + rayon adv
	//threshold_LR_mm = 140+40+150
	sensors_.addThresholdFront(450, 140 + 240 + 40 + rayon_adv, 450);
	sensors_.addThresholdFrontVeryClosed(200, 140 + 80 + 40 + rayon_adv, 200); //40 = patch cho

	sensors_.addThresholdBack(0, 140 + 240 + rayon_adv, 0);
	sensors_.addThresholdBackVeryClosed(200, 140 + 80 + rayon_adv, 200);

}
