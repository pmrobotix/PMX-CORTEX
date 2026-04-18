/*
 * INO_t41_ToFBeacon_fusion.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"
#include <i2c_register_slave.h>

//Variables globales

int ii = 0;
extern int video_infinite;

void setup()
{
	delay(3000);
	Serial.begin(921600); //fast serial
	Wire.setClock(1000000); // use fast mode I2C
	Wire.begin();
	Wire1.setClock(1000000); // use fast mode I2C
	Wire1.begin();

	// Just to know which program is running on my Arduino
	Serial.println(F("START " __FILE__));

	// initialize the digital pin as an output for LEDS
	pinMode(LED_BUILTIN, OUTPUT); // Green
	pinMode(LED_BUILTIN + 1, OUTPUT); //Red
	//pinMode(LED_BUILTIN + 2, OUTPUT); //optional replace by ADC

	setup_screen_splash();  // logo PM-ROBOTIX immédiat

	ledPanels_setup();

	tof_setup();

	setup_screen_menu();   // menu pre-match remplace le logo


}

void loop()
{
	// Reboot vers HalfKay sur commande série (pour upload PIO sans bouton reset)
	if (Serial.available()) {
		char c = Serial.read();
		if (c == 'R') {
			Serial.println("Rebooting to bootloader...");
			Serial.flush();
			delay(50);
			_reboot_Teensyduino_();
		}
	}

	ii++;
	if (ii % 2)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		digitalWrite(LED_BUILTIN + 1, HIGH);
	} else
	{
		digitalWrite(LED_BUILTIN, LOW);
		digitalWrite(LED_BUILTIN + 1, LOW);
	}


	screen_loop();

	ledPanels_loop(false);
	//Serial.println("loop");
	tof_loop(!video_infinite);

}
