/*
 * INO_ToF_DetectionBeacon.h
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#ifndef _INO_ToF_DetectionBeacon_H_
#define _INO_ToF_DetectionBeacon_H_
#include "Arduino.h"
#include <i2c_driver_wire.h>//
//#include <Wire.h>
#include "LedPanels.h"
#include "TofSensors.h"
#include "LCDScreen.h"

//#define SENSORS_VL_CLOSED_COLLISION_ACTIVATED 1

//Debug pour savoir a tout moment si un VL53 est out or not
#define DEBUG_VL_SUR_LEDMATRIX           1

//CONFIG TOF VL53
#define TotalWidthOfSPADS               16
#define WidthOfSPADsPerZone             4
#define NumOfSPADsShiftPerZone          4 //decalage de zone
#define NumOfSPADsToStartZone           1 //SENS normal:declage de 2; sens inversé du tableau centre :decalage de 1
#define NumOfZonesPerSensor             (((TotalWidthOfSPADS - WidthOfSPADsPerZone) / NumOfSPADsShiftPerZone) + 1)
#define NumOfSensors                    18
#define NumOfCollisionSensors           4

#define NumOfSensorsForVideoMode        26

#endif /* _INO_ToF_DetectionBeacon_H_ */
