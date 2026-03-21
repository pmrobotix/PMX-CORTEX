/*
 * TofSensors.cpp
 *
 *  Created on: Jan 25, 2021
 *      Author: cho (PM-ROBOTIX)
 */

#include "INO_ToF_DetectionBeacon.h"
#include "TofSensors.h"
#include <i2c_register_slave.h>

//Nouvelle config avec detecteurs
// front[9] back[9]
//front[9]:Violet41-Jaune40-Orange39-Rouge38-Marron37-Noir36-Blanc35-Gris34-violet33
//Back[9]:Violet20-Jaune21-Orange22-Rouge23-Marron4-Noir3-Blanc2-Gris1-violet0
int shutd_pin[NumOfSensors] = { 23, 22, 21, 20, 0, 1, 2, 3, 4, 32, 31, 30, 29, 28, 27, 26, 6, 5 };

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
//front[4]= BAS HAUT BAS HAUT  back[4]
int shutd_pin_collision[NumOfCollisionSensors + NumOfCollisionSensors] = { 40, 39, 38, 37, 36, 35, 34, 33 };
#endif

//centre des SPADs d'après le tableau (inversé de gauche vers la droite à cause du sens des VL)
int center[16] = { 251, 243, 235, 227, 219, 211, 203, 195, 187, 179, 171, 163, 155, 147, 139, 131 };

char buffer[255];

SFEVL53L1X vl[NumOfSensors];
VL53L1X_Result_t res[NumOfZonesPerSensor * NumOfSensors];

Threads::Mutex registers_new_data_lock;
Threads::Mutex filteredResultWorkingCopy_mutex;
Threads::Mutex l1_mutex;
Threads::Mutex l2_mutex;

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
SFEVL53L1X vl_collision[NumOfCollisionSensors + NumOfCollisionSensors];
VL53L1X_Result_t res_collision[NumOfCollisionSensors + NumOfCollisionSensors];
uint16_t filteredResult_coll[NumOfCollisionSensors + NumOfCollisionSensors];
uint16_t distance_coll[NumOfCollisionSensors + NumOfCollisionSensors];
uint8_t status_coll[NumOfCollisionSensors + NumOfCollisionSensors];
bool connected_coll[NumOfCollisionSensors + NumOfCollisionSensors];
uint16_t NumSPADs_coll[NumOfZonesPerSensor * NumOfSensors];
uint16_t SigPerSPAD_coll[NumOfZonesPerSensor * NumOfSensors];
uint16_t Ambient_coll[NumOfZonesPerSensor * NumOfSensors];
#endif

int nb_active_filtered_sensors;
int nb_active_filtered_sensors_mode2;
volatile int videoMode;
extern int video_infinite;
int latency_thread_error = 0;
int latency = 0;

//initialise the settings
// Register 0. Number of Robot which may to be detected, default 3.
// Register 1. Writable. Sets the mode for led display. 0 => OFF. 100 => FULL ON. 50 => Half luminosity.
// Register 2. Points à afficher
// Register 2. NOT use yet
Settings settings = { 0x03, 0x01, 0x00, 0x00 };

Registers registers; //data pret pour la demande I2C
I2CRegisterSlave registerSlave = I2CRegisterSlave(Slave2, (uint8_t*) &settings, sizeof(Settings), (uint8_t*) &registers,
		sizeof(Registers));

uint16_t filteredResult[NumOfZonesPerSensor * NumOfSensors];
uint16_t filteredResultPrevious[NumOfZonesPerSensor * NumOfSensors];
uint16_t filteredResultWorkingCopy[NumOfZonesPerSensor * NumOfSensors];
uint16_t distance_t[NumOfZonesPerSensor * NumOfSensors];
uint16_t greenHandDistance[NumOfZonesPerSensor * NumOfSensors]; //working result utilisé pour l'affichage LED
uint8_t status_t[NumOfZonesPerSensor * NumOfSensors];
bool connected_t[NumOfZonesPerSensor * NumOfSensors];
uint16_t NumSPADs_t[NumOfZonesPerSensor * NumOfSensors];
uint16_t SigPerSPAD_t[NumOfZonesPerSensor * NumOfSensors];
uint16_t Ambient_t[NumOfZonesPerSensor * NumOfSensors];

uint16_t handDistance[NumOfZonesPerSensor * NumOfSensors]; //working for detection

//uint16_t OffsetCal[NumOfZonesPerSensor * 9] = { 60, 110, 134, 95, 60, 110, 134,
//		95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134,
//		95, 60, 110, 134, 95, 60, 110, 134, 95, 60, 110, 134, 95 };
//uint16_t OffsetCalxTalk[NumOfZonesPerSensor * 9] = { 0, 85, 31, 117, 0, 85, 31,
//		117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117, 0,
//		85, 31, 117, 0, 85, 31, 117, 0, 85, 31, 117 };

volatile bool tofVLReadyForCalculation = false;
//volatile bool tofVLunblock = false;
volatile int shared_endloop1 = 0;
volatile int shared_endloop2 = 0;

//elapsedMicros elapsedT_us = 0;

//Calcul et filtrage
int8_t calculPosition(float decalage_deg, Registers &new_values, uint16_t *fresult)
{

	int filtrage_mm = 10; //attention si on mets plus, par ex 100mm, certaines valeurs sont à zero avec la balise en face et très prets, voir si un calibrage regle le problème  lorsque l'on approche les mains

	int max_nb_bots = 4; //TODO utiliser les settings

	int final_nb_bots = 0;

	uint16_t tab[9 * max_nb_bots] = { 0 }; // données des 4 balises z1_p, z1_1=>z1_5
	float pos[3 * max_nb_bots] = { 0.0 };
	uint16_t dist[max_nb_bots] = { 0 };
	for (int i = 0; i < 9 * max_nb_bots; i++)
	{
		tab[i] = 0;
	}
	for (int i = 0; i < 3 * max_nb_bots; i++)
	{
		pos[i] = 0.0;
	}
	for (int i = 0; i < max_nb_bots; i++)
	{
		dist[i] = 0;
	}

	int index_tab_bot = 0;

	int p = -1;
	bool end_ = false;

	int chevauchement_arriere_nb = 0;

	//1. déterminer les 4 balises pour log (chevauchement data possible)
	for (int nb_bots = 1; nb_bots <= max_nb_bots; nb_bots++)
	{

		//on parcours pour trouver une balise (0 à 71)
		for (int n = p + 1; n < (NumOfZonesPerSensor * NumOfSensors); n++)
		{

			uint16_t val = fresult[n];
			if (val > 25) //filtre distance min
			{ //filtrage_mm) { //on prend que la premiere detection au dessus de 100mm pour filtrer la main qui s'approche
			  //on a trouvé une balise
				final_nb_bots = nb_bots;

				//Serial.println("beacon found " + String(n) + " " + String(val));
				if (index_tab_bot < 9 * max_nb_bots)
				{
					tab[0 + index_tab_bot] = n;
					tab[1 + index_tab_bot] = 1;
					tab[2 + index_tab_bot] = val;
				}
				//gestion du chevauchement avec filtrage
				if (n == 0) if (fresult[0] > filtrage_mm)
				{
					if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 1] > filtrage_mm)
					{
						chevauchement_arriere_nb = 1;
						if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 2] > filtrage_mm)
						{
							chevauchement_arriere_nb = 2;
							if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 3] > filtrage_mm)
							{
								chevauchement_arriere_nb = 3;
								if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 4] > filtrage_mm)
								{
									chevauchement_arriere_nb = 4;
									if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 5] > filtrage_mm)
									{
										chevauchement_arriere_nb = 5;
										if (fresult[(NumOfZonesPerSensor * NumOfSensors) - 6] > filtrage_mm)
										{
											chevauchement_arriere_nb = 6;
										}
									}
								}
							}
						}
					}
				}

				p = n; //enregistrement du dernier
				if (index_tab_bot < 9 * max_nb_bots)
				{
					if (n <= (NumOfZonesPerSensor * NumOfSensors) - 2)
						if (fresult[n + 1] > 1 && (abs(fresult[n + 1] - fresult[n]) < 250))
						{
							tab[3 + index_tab_bot] = fresult[n + 1];
							p = n + 1;
							tab[1 + index_tab_bot] = 2;
							if (n <= (NumOfZonesPerSensor * NumOfSensors) - 3)
								if (fresult[n + 2] > 1 && (abs(fresult[n + 2] - fresult[n + 1]) < 250))
								{
									tab[4 + index_tab_bot] = fresult[n + 2];
									p = n + 2;
									tab[1 + index_tab_bot] = 3;
									if (n <= (NumOfZonesPerSensor * NumOfSensors) - 4)
										if (fresult[n + 3] > 1 && (abs(fresult[n + 3] - fresult[n + 2]) < 250))
										{
											tab[5 + index_tab_bot] = fresult[n + 3];
											p = n + 3;
											tab[1 + index_tab_bot] = 4;
											if (n <= (NumOfZonesPerSensor * NumOfSensors) - 5)
												if (fresult[n + 4] > 1 && (abs(fresult[n + 4] - fresult[n + 3]) < 250))
												{
													tab[6 + index_tab_bot] = fresult[n + 4];
													p = n + 4;
													tab[1 + index_tab_bot] = 5;
													if (n <= (NumOfZonesPerSensor * NumOfSensors) - 6)
														if (fresult[n + 5] > 1
																&& (abs(fresult[n + 5] - fresult[n + 4]) < 250))
														{
															tab[7 + index_tab_bot] = fresult[n + 5];
															p = n + 5;
															tab[1 + index_tab_bot] = 6;
															if (n <= (NumOfZonesPerSensor * NumOfSensors) - 7)
																if (fresult[n + 6] > 1
																		&& (abs(fresult[n + 6] - fresult[n + 5]) < 250))
																{
																	tab[8 + index_tab_bot] = fresult[n + 6];
																	p = n + 6;
																	tab[1 + index_tab_bot] = 7;
																}
														}
												}
										}
								}
						}
				}
				break;
			} else
			{
				if (n >= (NumOfZonesPerSensor * NumOfSensors) - 1)
				{
					end_ = true;
					break;
				}
			}
		}
		if (end_) break;
		if (p == (NumOfZonesPerSensor * NumOfSensors) - 1) break; //cas d'arret avec chevauchement
		index_tab_bot += 9;
	}
	int c = 0;

	//gestion du chevauchement de data < 0, on ne retient que max_balises que l'on reduit à max-1
	if (chevauchement_arriere_nb > 0)
	{
		//on merge la balise 1 et la dernière (last)
		for (int i = 2; i < 9; i++)
		{
			if (tab[i + 9 * (final_nb_bots - 1)] == 0) //anciennement -1
			{
				if (c == 0) c = i;
				if (tab[i - c + 2] != 0) //anciennement -1
				{
					tab[i + 9 * (final_nb_bots - 1)] = tab[i - c + 2];
					tab[1 + 9 * (final_nb_bots - 1)]++;
				}
			}
		}
		//on decale
		for (int i = 9; i < 9 * max_nb_bots; i++)
		{
			tab[i - 9] = tab[i];
		}
		//on efface le last
		for (int i = 9 * (max_nb_bots - 1); i < 9 * max_nb_bots; i++)
		{
			tab[i] = 0; //anciennement -1
		}
		final_nb_bots--;
	}

	//TODO ne garder que les balises detectees au dessus de 150mm

	//TODO semaphore

	//sauvegarde les logs des 4 balises
	new_values.z1_p = (uint8_t) tab[0];
	new_values.z1_n = (uint8_t) tab[1];
	new_values.z1_1 = tab[2];
	new_values.z1_2 = tab[3];
	new_values.z1_3 = tab[4];
	new_values.z1_4 = tab[5];
	new_values.z1_5 = tab[6];
	new_values.z1_6 = tab[7];
	new_values.z1_7 = tab[8];

	new_values.z2_p = (uint8_t) tab[9];
	new_values.z2_n = (uint8_t) tab[10];
	new_values.z2_1 = tab[11];
	new_values.z2_2 = tab[12];
	new_values.z2_3 = tab[13];
	new_values.z2_4 = tab[14];
	new_values.z2_5 = tab[15];
	new_values.z2_6 = tab[16];
	new_values.z2_7 = tab[17];

	new_values.z3_p = (uint8_t) tab[18];
	new_values.z3_n = (uint8_t) tab[19];
	new_values.z3_1 = tab[20];
	new_values.z3_2 = tab[21];
	new_values.z3_3 = tab[22];
	new_values.z3_4 = tab[23];
	new_values.z3_5 = tab[24];
	new_values.z3_6 = tab[25];
	new_values.z3_7 = tab[26];

	new_values.z4_p = (uint8_t) tab[27];
	new_values.z4_n = (uint8_t) tab[28];
	new_values.z4_1 = tab[29];
	new_values.z4_2 = tab[30];
	new_values.z4_3 = tab[31];
	new_values.z4_4 = tab[32];
	new_values.z4_5 = tab[33];
	new_values.z4_6 = tab[34];
	new_values.z4_7 = tab[35];

//2. pour chaque balise déterminer la zone centre et le capteur concerné
	//360 / 72 = 5 degrés par zone

	//float zone_begin_angle_deg[9*4] = { 0.5 , 0.5+4.8 , 0.5+4.8+4.8 , 0.5+4.8+4.8+4.8}; //+20
	//float zone_end_angle_deg[9*4] = { 0.5+4.8 , 0.5+4.8+4.8 , 0.5+4.8+4.8+4.8, 0.5+4.8+4.8+4.8}; //+20

	//pour chaque balise detectée
	for (int i = 1; i <= final_nb_bots; i++)
	{

		uint16_t num_zone_begin = tab[0 + 9 * (i - 1)];
		uint16_t num_zone_end = num_zone_begin + tab[1 + 9 * (i - 1)] - 1;

		float zone_begin_angle_deg = (num_zone_begin / 4) * 20.0 + 0.5 + ((num_zone_begin % 4) * 4.8);
		float zone_end_angle_deg = (num_zone_end / 4) * 20.0 + 0.5 + ((num_zone_end % 4) * 4.8) + 4.8;

		//determiné l'angle milieu
		float milieu_deg = (zone_begin_angle_deg + zone_end_angle_deg) / 2.0;
		//si depassement audessus des 72 zones, on reduit d'un tour
		if (milieu_deg > 360)
		{ //num_zone_end > 71 &&
			milieu_deg -= 360;
		}

		pos[2 + 3 * (i - 1)] = milieu_deg + decalage_deg;

//        Serial.print("CALCUL Balise" + String(i));
//        Serial.print(" num_zone_begin = " + String(num_zone_begin));
//        Serial.print(" num_zone_end = " + String(num_zone_end));
//        Serial.print(" zone_begin_angle_deg = " + String(zone_begin_angle_deg));
//        Serial.print(" zone_end_angle_deg = " + String(zone_end_angle_deg));
//        Serial.print(" milieu_deg = " + String(milieu_deg));
//        Serial.println();
	}

//3. déterminer la distance sur le repère de la balise (= identique au robot)

	//TODO filtrage de la distance sur 2 balises cote à cote, si mm dist alors c'est la meme balise
	//calcul des distances

	//pour chaque balise detectée calcul de la distance

	int offset_mm = 40 + 80 + 30;
	for (int i = 1; i <= final_nb_bots; i++)
	{
		int m = 0;
		int sum = 0;
		int min = 9999;
		for (int z = 2 + 9 * (i - 1); z <= 7 + 9 * (i - 1); z++)
		{
			//moy de tab[2] à tab[7]
			if (tab[z] > filtrage_mm)
			{
				if (tab[z] < min) min = tab[z];
				sum += tab[z];
				m++;
			}
		}
		if (m > 2)
		{
			m--;
			sum -= min;
		}
		float moy = (1.0 * sum / m) + offset_mm;
		dist[i - 1] = moy;// + 100; //centre à centre //TODO ne fonctionne pas, ca donne la distance entre les capteur et le catadiopthre

//        Serial.print(" moy(center)" + String(i));
//        Serial.print(" = " + String(dist[i - 1]));
//        Serial.print("  ");
	}
//    Serial.println();

	//pour chaque balise detectée calcul des coordonnées x,y
	//avec simplification et arrondi trigonometrique avec un seul sin/cos
	for (int i = 1; i <= final_nb_bots; i++)
	{
		float x = (dist[i - 1]) * cos(pos[2 + 3 * (i - 1)] * PI / 180.0);
		float y = (dist[i - 1]) * sin(pos[2 + 3 * (i - 1)] * PI / 180.0);
		pos[0 + 3 * (i - 1)] = x;
		pos[1 + 3 * (i - 1)] = y;

//        Serial.print("coord" + String(i));
//        Serial.print(" x=" + String(x));
//        Serial.print(" y=" + String(y));
//        Serial.print("  ");
	}
//    Serial.println();

	//enregistrement

	new_values.x1_mm = (int16_t) pos[0];
	new_values.y1_mm = (int16_t) pos[1];
	new_values.a1_deg = pos[2];
	new_values.x2_mm = (int16_t) pos[3];
	new_values.y2_mm = (int16_t) pos[4];
	new_values.a2_deg = pos[5];
	new_values.x3_mm = (int16_t) pos[6];
	new_values.y3_mm = (int16_t) pos[7];
	new_values.a3_deg = pos[8];
	new_values.x4_mm = (int16_t) pos[9];
	new_values.y4_mm = (int16_t) pos[10];
	new_values.a4_deg = pos[11];
	new_values.d1_mm = dist[0];
	new_values.d2_mm = dist[1];
	new_values.d3_mm = dist[2];
	new_values.d4_mm = dist[3];

	return final_nb_bots;
}

int scani2c(TwoWire w)
{
	Serial.println("I2C scanner. Scanning ...");
	int count = 0;
	for (byte i = 1; i < 120; i++)
	{
		w.beginTransmission(i);
		if (w.endTransmission() == 0)
		{
			//Serial.print("Found address: ");
			Serial.print(i, DEC);
			Serial.print(" (0x");
			Serial.print(i, HEX);
			Serial.print(") ; ");
			count++;
			delay(1);
		}
	}
	Serial.println("Done.");
	Serial.print("Found ");
	Serial.print(count, DEC);
	Serial.println(" device(s).");
	return count;
}

void on_read_isr(uint8_t reg_num)
{
	// Clear the "new data" bit so the master knows it's
	// already read this set of values.
	registers.flags = registers.flags & 0xFE; //mise a zero du BIT0
}

void tof_setup()
{
	//threads.delay(4000);
	videoMode = 0;

	// Start listening on I2C4 with address 0x2D
	registerSlave.listen(0x2D);
	registerSlave.after_read(on_read_isr);

	//Front vl on i2c  17 SDA1 / 16 SCL1
	for (int i = 0; i < NumOfSensors / 2; i++)
	{
		vl[i] = SFEVL53L1X(Wire1, shutd_pin[i], -1);
	}
	//Back vl on i2c  18 SDA / 19 SCL
	for (int i = NumOfSensors / 2; i < NumOfSensors; i++)
	{
		vl[i] = SFEVL53L1X(Wire, shutd_pin[i], -1);
	}
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
    //config collision Front vl on i2c 18 SDA / 19 SCL
    for (int i = 0; i < (NumOfCollisionSensors); i++) {
        vl_collision[i] = SFEVL53L1X(Wire, shutd_pin_collision[i], -1);
    }
    //config collision Back vl on i2c  17 SDA1 / 16 SCL1
    for (int i = NumOfCollisionSensors; i < (NumOfCollisionSensors + NumOfCollisionSensors); i++) {
        vl_collision[i] = SFEVL53L1X(Wire1, shutd_pin_collision[i], -1);
    }
#endif
	//Config all shutpin
	for (int i = 0; i < NumOfSensors; i++)
	{
		pinMode(shutd_pin[i], OUTPUT);
	}
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
    for (int i = 0; i < NumOfCollisionSensors + NumOfCollisionSensors; i++) {
        pinMode(shutd_pin_collision[i], OUTPUT);
    }
#endif
	//deactivate all
	for (int i = 0; i < NumOfSensors; i++)
	{
		digitalWrite(shutd_pin[i], LOW);
	}
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
    for (int i = 0; i < NumOfCollisionSensors + NumOfCollisionSensors; i++) {
        digitalWrite(shutd_pin_collision[i], LOW);
    }
#endif
	Serial.println("vl I2C Change address ...");
//    I2C scanner. Scanning ...
//    21 (0x15) ; 22 (0x16) ; 23 (0x17) ; 24 (0x18) ; 25 (0x19) ; 26 (0x1A) ; 27 (0x1B) ; 28 (0x1C) ; 29 (0x1D) ; Done.
//    Found 9 device(s).
//    I2C scanner. Scanning ...
//    30 (0x1E) ; 31 (0x1F) ; 32 (0x20) ; 33 (0x21) ; 34 (0x22) ; 35 (0x23) ; 38 (0x26) ; 43 (0x2B) ; 44 (0x2C) ; 45 (0x2D) ; 46 (0x2E) ; Done.
//    Found 11 device(s).
//    Config VL (FRONT):0=0x15 1=0x16 2=0x17 3=0x18 4=0x19 5=0x1A 6=0x1B 7=0x1C 8=0x1D
//    Config VL  (BACK):9=0x1E 10=0x1F 11=0x20 12=0x21 13=0x22 14=0x23 15=[0x29 ERROR] 16=[0x29 ERROR] 17=0x26
//    Config VL  (COLL):0=[0x29 ERROR] 1=[0x29 ERROR] 2=[0x29 ERROR] 3=[0x29 ERROR] 4=0x2B 5=0x2C 6=0x2D 7=0x2E

	for (int i = 0; i < NumOfSensors; i++)
	{
		pinMode(shutd_pin[i], INPUT);
		delay(25);
		if (vl[i].begin() == 0)
		{
			vl[i].setI2CAddress((uint8_t) ((0x15 + i) << 1)); //(DEC=21 22 23 ...) //address on 7bits<<1
			digitalWrite(shutd_pin[i], HIGH);
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				connected_t[(NumOfZonesPerSensor * i) + z] = true;
			}
		} else
		{
			Serial.print("ERROR vl[" + String(i) + "] OFFLINE! ");
			Serial.print(" 0x");
			Serial.println((0x15 + i), HEX);
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				connected_t[(NumOfZonesPerSensor * i) + z] = false;
			}
		}
	}
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
    Serial.println("vl collision I2C Change address ...");

    for (int i = 0; i < (NumOfCollisionSensors + NumOfCollisionSensors); i++) {
        pinMode(shutd_pin_collision[i], INPUT);
        delay(20);
        if (vl_collision[i].begin() == 0) {
            vl_collision[i].setI2CAddress((uint8_t) ((0x30 + i) << 1));  //(HEX=27 28 29 2A ...) //address on 7bits<<1
            digitalWrite(shutd_pin_collision[i], HIGH);

            connected_coll[i] = true;
        }
        else {
            Serial.print("ERROR vl_collision[" + String(i) + "] OFFLINE! ");
            Serial.print(" 0x");
            Serial.println((0x30 + i), HEX);

            connected_coll[i] = false;
        }
    }
#endif

	Serial.print("Config VL (FRONT):");
	//configuration de chaque VL
	for (int n = 0; n < NumOfSensors; n++)
	{
		if (connected_t[n])
		{
			int available = vl[n].checkID();
			if (available)
			{
				//connected_t[n] = true;
				int addr = vl[n].getI2CAddress() >> 1;
				Serial.print(String(n) + "=0x");
				Serial.print(addr, HEX);
				Serial.print(" ");
				//uint16_t sensorId = vl[n].getSensorID();

				vl[n].setDistanceModeShort(); //Short to have relevant figures //long takes too much time!
				vl[n].setTimingBudgetInMs(TIMING_UDGET_IN_MS); //15
				vl[n].setIntermeasurementPeriod(TIMING_UDGET_IN_MS + 1); //16 measure periodically. Intermeasurement period must be >/= timing budget.
				if (n == 8)
				{
					Serial.println();
					Serial.print("Config VL  (BACK):");
				}
			} else
			{
				Serial.print(String(n) + "=[0x");
				Serial.print(vl[n].getI2CAddress() >> 1, HEX);
				Serial.print(" ERROR");
				Serial.print("] ");

				for (int z = 0; z < NumOfZonesPerSensor; z++)
				{
					connected_t[(NumOfZonesPerSensor * n) + z] = false;
				}
				status_t[n] = 8;
			}
		}
	}
	Serial.println();
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
    //Configuration de chaque VL COLLISION
    Serial.print("Config VL  (COLL):");
    for (int n = 0; n < (NumOfCollisionSensors + NumOfCollisionSensors); n++) {
        if (connected_coll[n]) {
            int available = vl_collision[n].checkID();
            if (available) {
                //connected_coll[n] = true;
                int addr = vl_collision[n].getI2CAddress() >> 1;
                Serial.print(String(n) + "=0x");
                Serial.print(addr, HEX);
                Serial.print(" ");
                //uint16_t sensorId = vl_collision[n].getSensorID();

                vl_collision[n].setDistanceModeShort(); //Short to have relevant figures
                vl_collision[n].setTimingBudgetInMs(TIMING_UDGET_IN_MS);
                vl_collision[n].setIntermeasurementPeriod(TIMING_UDGET_IN_MS+1); //51 measure periodically. Intermeasurement period must be >/= timing budget.
                if (n == 8) Serial.println();
            }
            else {
                Serial.print(String(n) + "= ");
                //Serial.print(vl_collision[n].getI2CAddress() >> 1, HEX);
                Serial.print(" ERROR");
                Serial.print("] ");

                connected_coll[n] = false;

                status_coll[n] = 8;
            }
        }
    }
    Serial.println();

#endif

    //delay
    	threads.delay(1000);
	//scan les bus i2c pour verifier les adresses i2c
	scani2c(Wire1); //front
	scani2c(Wire); //back
	// TODO faire qqch si le count n'est aps 2x9 ?

	//delay
	threads.delay(1000);

	//On demarre les 2 threads de sensors
	threads.addThread(loopvl1);
	threads.addThread(loopvl2);
	//tofVLReadyForCalculation = true; //pour démarrer la synchro
}

//Attention pas de delay dans cette fonction pour ne pas ralentir la main loop
void tof_loop(int debug)
{
	elapsedMicros elapsedT_us = 0;

	long t_start = elapsedT_us;

	//TODO si l'un VL est deconnecté
	//TODO si Off/ON alors reinit
	//TODO si perte de signal sur un des cables i2C
	//TODO passage de la balise une partie en debut et à la fin

	//	attente synchronisation des DATA

	shared_endloop1 =0;
	shared_endloop2 =0;
	tofVLReadyForCalculation = false;

	while (!shared_endloop1 | !shared_endloop2) // les 2 threads ont terminés
	{
		threads.yield();
	}

//	while (!tofVLReadyForCalculation)
//	{
//		threads.yield();
//		Serial.print(" n");
//	}
//	while (tofVLReadyForCalculation)
//	{
//		threads.yield();
//		Serial.println(" OK");
//	}

	long t_endwaitthreads = elapsedT_us;

	//Traitement et copie DATA

	//algo de correction primaire

	//1. Cas du zero sur la distance qui arrive de temps en temps, on utilise l'ancienne donnée si elle existe
	for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
	{
		if (filteredResult[n] == 0)
		{
			if (filteredResultPrevious[n] != 0) filteredResult[n] = filteredResultPrevious[n];
		}
	}

	//test du 0  0 tout tout pres
	for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
	{
		if (filteredResult[n] == 0)
		{
			filteredResult[n] = 30;
		}
	}

	//todo FAIRE LA TRANSITION INF A 3 ET MAX -3

	//3. sur plusieurs valeurs, si SigPerSPAD_t < 5, on ne garde que ces valeurs <5
	for (int n = 3; n < (NumOfSensors * NumOfZonesPerSensor) - 3; n++)
	{
		if ((filteredResult[n] > (filteredResult[n - 3] - 50)) & (filteredResult[n] < (filteredResult[n - 3] + 50)))
		{
			if ((status_t[n - 3] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{
				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n - 3] = 1;
				}

			}
		}
		if ((filteredResult[n] > (filteredResult[n - 2] - 50)) & (filteredResult[n] < (filteredResult[n - 2] + 50)))
		{
			if ((status_t[n - 2] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{
				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n - 2] = 1;
				}

			}
		}
		if ((filteredResult[n] > (filteredResult[n - 1] - 50)) & (filteredResult[n] < (filteredResult[n - 1] + 50)))
		{
			if ((status_t[n - 1] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{

				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n - 1] = 1;
				}

			}
		}
		if ((filteredResult[n] > (filteredResult[n + 1] - 50)) & (filteredResult[n] < (filteredResult[n + 1] + 50)))
		{
			if ((status_t[n + 1] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{
				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n + 1] = 1;
				}

			}
		}
		if ((filteredResult[n] > (filteredResult[n + 2] - 50)) & (filteredResult[n] < (filteredResult[n + 2] + 50)))
		{
			if ((status_t[n + 2] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{
				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n + 2] = 1;
				}
			}

		}
		if ((filteredResult[n] > (filteredResult[n + 3] - 50)) & (filteredResult[n] < (filteredResult[n + 3] + 50)))
		{
			if ((status_t[n + 3] == PhaseOutOfLimit)
					& (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
			{
				if (SigPerSPAD_t[n] > 14000)
				{
					filteredResult[n + 3] = 1;
				}
			}

		}
	}
	//3.greenHandDistance
	for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
	{
		if ((status_t[n] == RangeValid) & (SigPerSPAD_t[n] > 1000) & (NumSPADs_t[n] > 10)) //| (status_t[n] == SignalFail)| (status_t[n] == PhaseOutOfLimit))
		{

			greenHandDistance[n] = distance_t[n];

		} else
		{
			greenHandDistance[n] = 0;
		}
	}

	//4.sauvegarde de l'ancienne valeur
	for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
	{
		if (filteredResult[n] >= 0) filteredResultPrevious[n] = distance_t[n];
	}


//on copie les données (avec semaphore)
	filteredResultWorkingCopy_mutex.lock();
	memcpy(&filteredResultWorkingCopy, &filteredResult, sizeof(filteredResult));
	filteredResultWorkingCopy_mutex.unlock();

	//Les DONNÉES sont PRETES : on peut lancer les threads pendant les calculs
	tofVLReadyForCalculation = true;

	//on peut relancer les threads en avance
	shared_endloop1 =0;
	shared_endloop2 =0;




	// Gather raw data and convert to output values
	Registers new_values;

	//Calcul des positions
	float decalage_deg = -22.0;
	filteredResultWorkingCopy_mutex.lock();
	new_values.nbDetectedBots = calculPosition(decalage_deg, new_values, filteredResultWorkingCopy);
	filteredResultWorkingCopy_mutex.unlock();

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
	    //Save data into new_values
		new_values.c1_mm = filteredResult_coll[0]; //TODO COPY ?
		new_values.c2_mm = filteredResult_coll[1];
		new_values.c3_mm = filteredResult_coll[2];
		new_values.c4_mm = filteredResult_coll[3];
		new_values.c5_mm = filteredResult_coll[4];
		new_values.c6_mm = filteredResult_coll[5];
		new_values.c7_mm = filteredResult_coll[6];
		new_values.c8_mm = filteredResult_coll[7];

#endif

	registers_new_data_lock.lock();
	// Block copy new values over the top of the old values
	// and then set the "new data" bit.
	//registers.flags = 1 && 0x01;
	registers.flags = registers.flags | 0x01;
	memcpy(&registers, &new_values, sizeof(Registers));
	registers_new_data_lock.unlock();

	long t_endcalculation = elapsedT_us;

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
	    Serial.print("FRONT: ");
	    Serial.print(new_values.c1_mm);
	    Serial.print(" ");
	    Serial.print(new_values.c2_mm);
	    Serial.print(" ");
	    Serial.print(new_values.c3_mm);
	    Serial.print(" ");
	    Serial.print(new_values.c4_mm);
	    Serial.print(" BACK: ");
	    Serial.print(new_values.c5_mm);
	    Serial.print(" ");
	    Serial.print(new_values.c6_mm);
	    Serial.print(" ");
	    Serial.print(new_values.c7_mm);
	    Serial.print(" ");
	    Serial.println(new_values.c8_mm);
	#endif

	Serial.print("  FLAGS: ");
	Serial.print(new_values.flags);
	Serial.print(" NBBOTS: ");
	Serial.print(new_values.nbDetectedBots);

	Serial.print(" xyad1: ");
	Serial.print(new_values.x1_mm);
	Serial.print(" ");
	Serial.print(new_values.y1_mm);
	Serial.print(" ");
	Serial.print(new_values.a1_deg);
	Serial.print(" ");
	Serial.print(new_values.d1_mm);
	Serial.print(" xyad2: ");
	Serial.print(new_values.x2_mm);
	Serial.print(" ");
	Serial.print(new_values.y2_mm);
	Serial.print(" ");
	Serial.print(new_values.a2_deg);
	Serial.print(" ");
	Serial.print(new_values.d2_mm);
	Serial.print(" xyad3: ");
	Serial.print(new_values.x3_mm);
	Serial.print(" ");
	Serial.print(new_values.y3_mm);
	Serial.print(" ");
	Serial.print(new_values.a3_deg);
	Serial.print(" ");
	Serial.print(new_values.d3_mm);
	Serial.print(" xyad4: ");
	Serial.print(new_values.x4_mm);
	Serial.print(" ");
	Serial.print(new_values.y4_mm);
	Serial.print(" ");
	Serial.print(new_values.a4_deg);
	Serial.print(" ");
	Serial.print(new_values.d4_mm);

	Serial.println();
	Serial.print("  R1:");
	Serial.print(new_values.z1_p);
	Serial.print("(");
	Serial.print(new_values.z1_n);
	Serial.print("): ");
	Serial.print(new_values.z1_1);
	Serial.print(" ");
	Serial.print(new_values.z1_2);
	Serial.print(" ");
	Serial.print(new_values.z1_3);
	Serial.print(" ");
	Serial.print(new_values.z1_4);
	Serial.print(" ");
	Serial.print(new_values.z1_5);
	Serial.print(" ");
	Serial.print(new_values.z1_6);
	Serial.print(" ");
	Serial.print(new_values.z1_7);
	Serial.print("  R2:");
	Serial.print(new_values.z2_p);
	Serial.print("(");
	Serial.print(new_values.z2_n);
	Serial.print("): ");
	Serial.print(new_values.z2_1);
	Serial.print(" ");
	Serial.print(new_values.z2_2);
	Serial.print(" ");
	Serial.print(new_values.z2_3);
	Serial.print(" ");
	Serial.print(new_values.z2_4);
	Serial.print(" ");
	Serial.print(new_values.z2_5);
	Serial.print(" ");
	Serial.print(new_values.z2_6);
	Serial.print(" ");
	Serial.print(new_values.z2_7);
	Serial.print("  R3:");
	Serial.print(new_values.z3_p);
	Serial.print("(");
	Serial.print(new_values.z3_n);
	Serial.print("): ");
	Serial.print(new_values.z3_1);
	Serial.print(" ");
	Serial.print(new_values.z3_2);
	Serial.print(" ");
	Serial.print(new_values.z3_3);
	Serial.print(" ");
	Serial.print(new_values.z3_4);
	Serial.print(" ");
	Serial.print(new_values.z3_5);
	Serial.print(" ");
	Serial.print(new_values.z3_6);
	Serial.print(" ");
	Serial.print(new_values.z3_7);
	Serial.print("  R4:");
	Serial.print(new_values.z4_p);
	Serial.print("(");
	Serial.print(new_values.z4_n);
	Serial.print("): ");
	Serial.print(new_values.z4_1);
	Serial.print(" ");
	Serial.print(new_values.z4_2);
	Serial.print(" ");
	Serial.print(new_values.z4_3);
	Serial.print(" ");
	Serial.print(new_values.z4_4);
	Serial.print(" ");
	Serial.print(new_values.z4_5);
	Serial.print(" ");
	Serial.print(new_values.z4_6);
	Serial.print(" ");
	Serial.print(new_values.z4_7);
	Serial.println();

	//VIDEO MODE

	nb_active_filtered_sensors_mode2 = 0;
	nb_active_filtered_sensors = 0;
	for (int n = 0; n < NumOfSensors; n++)
	{
		for (int z = 0; z < NumOfZonesPerSensor; z++)
		{
			int val = filteredResultWorkingCopy[(NumOfZonesPerSensor * n) + z];
			int handdist = distance_t[(NumOfZonesPerSensor * n) + z];
			if ((handdist > 0) & (handdist < 160)) //1 est la valeur par defaut
				nb_active_filtered_sensors++;

			int handdist_mode2 = filteredResultWorkingCopy[(NumOfZonesPerSensor * n) + z];
			if ((handdist_mode2 > 10) & (handdist_mode2 < 60)) //1 est la valeur par defaut
				nb_active_filtered_sensors_mode2++;

			Serial.print(val);
			Serial.print(",");
			//Serial.print(" ");
		}
	}
	Serial.println();

	//change mode
	if (nb_active_filtered_sensors > NumOfSensorsForVideoMode)
	{
		videoMode = 1;
	} else if (nb_active_filtered_sensors_mode2 > 10)
	{
		videoMode = 2;

	} else
		videoMode = 0;
	Serial.println(videoMode);


	threads.delay(1);//important!!

	//PRINT INFO DEBUG //TODO WORKING COPY?
	if (debug)
	{
		uint16_t temp = 0;
		Serial.print("Working :");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				temp = filteredResultWorkingCopy[(NumOfZonesPerSensor * n) + z];
				if (temp != 1)
				{
					sprintf(buffer, "  %4d", temp);

				} else
					sprintf(buffer, "      ");
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("Number  :");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, "    %2d", (NumOfZonesPerSensor * n) + z);
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("Green  :");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				temp = greenHandDistance[(NumOfZonesPerSensor * n) + z];
				if (temp != 1)
				{
					sprintf(buffer, "  %4d", temp);

				} else
					sprintf(buffer, "      ");
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("Distance:");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, "  %4d", distance_t[(NumOfZonesPerSensor * n) + z]); //TODO WORKING COPY?
				Serial.print(buffer);
			}
		}

		Serial.println();

		Serial.print("status  :");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, "     %1d", status_t[(NumOfZonesPerSensor * n) + z]); //TODO WORKING COPY?
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("NumSPADs:");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, "    %2d", NumSPADs_t[(NumOfZonesPerSensor * n) + z]); //TODO WORKING COPY?
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("SPerSPAD:");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, " %5d", SigPerSPAD_t[(NumOfZonesPerSensor * n) + z]); //TODO WORKING COPY?
				Serial.print(buffer);
			}
		}
		Serial.println();

		Serial.print("Ambient :");
		for (int n = 0; n < NumOfSensors; n++)
		{
			memset(buffer, 0, strlen(buffer));
			for (int z = 0; z < NumOfZonesPerSensor; z++)
			{
				sprintf(buffer, " %5d", Ambient_t[(NumOfZonesPerSensor * n) + z]); //TODO WORKING COPY?
				Serial.print(buffer);
			}
		}
		Serial.println();

//        for (int n = 0; n < NumOfSensors; n++) {
//            memset(buffer, 0, strlen(buffer));
//            for (int z = 0; z < NumOfZonesPerSensor; z++) {
//                sprintf(buffer, "%01d  ", connected_t[(NumOfZonesPerSensor * n) + z]);
//                Serial.print(buffer);
//            }
//        }
//        Serial.println();

	}
	long t_endprintDebug = elapsedT_us;

	//print debug time
	Serial.println(
			" t_start=" + String(t_start) + " t_threads=" + String(t_endwaitthreads - t_start) + " t_calcul="
					+ String(t_endcalculation - t_endwaitthreads) + " t_print="
					+ String(t_endprintDebug - t_endcalculation));
	Serial.println();
	if (t_endwaitthreads - t_start > 90000)
	{
		latency_thread_error = 1;
		Serial.println(">>>>100000!!!!!!!!!");
		Serial.println();
		video_infinite = 1;

	} else
	{
		latency++;
		if (latency > 5)
		{
			latency_thread_error = 0;
			latency = 0;
		}
	}

	elapsedT_us = 0;
	//shared_endloop1 = 0; //lancement de la mise à jour des données loop1
	//shared_endloop2 = 0; //lancement de la mise à jour des données loop2

	threads.yield();
}

/**Table of Optical Centers**
 *
 * 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
 * 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
 * 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
 * 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
 * 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
 * 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
 * 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
 * 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

 * 127,119,111,103, 95, 87, 79, 71,  63, 55, 47, 39, 31, 23, 15, 7
 * 126,118,110,102, 94, 86, 78, 70,  62, 54, 46, 38, 30, 22, 14, 6
 * 125,117,109,101, 93, 85, 77, 69,  61, 53, 45, 37, 29, 21, 13, 5
 * 124,116,108,100, 92, 84, 76, 68,  60, 52, 44, 36, 28, 20, 12, 4
 * 123,115,107, 99, 91, 83, 75, 67,  59, 51, 43, 35, 27, 19, 11, 3
 * 122,114,106, 98, 90, 82, 74, 66,  58, 50, 42, 34, 26, 18, 10, 2
 * 121,113,105, 97, 89, 81, 73, 65,  57, 49, 41, 33, 25, 17, 9, 1
 * 120,112,104, 96, 88, 80, 72, 64,  56, 48, 40, 32, 24, 16, 8, 0 Pin 1
 *
 * To set the center, set the pad that is to the right and above the exact center of the region you'd like to measure as your opticalCenter*/

//Front loop
void loopvl1()
{
//	tofVLReadyForCalculation = false;
	shared_endloop1 = 1;
//	int ontime = 1;
	while (1)
	{
		//Serial.println("loopvl1");
		//tofVLunblock = false;
//		shared_endloop1 = 0; //start endloop2 en meme temps
		//tofVLReadyForCalculation = false;
		while (shared_endloop1 | shared_endloop2)
		{
			threads.yield();
		}
		for (int z = 0; z < NumOfZonesPerSensor; z++)
		{
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
            if (z == 0) //Gestion collision
                    {
                for (int c = 0; c < NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        vl_collision[c].setROI(16, 16, 199); //full matrix
                        vl_collision[c].startRanging();
                    }
                }
            }
#endif
			for (int n = 0; n < NumOfSensors / 2; n++)
			{ //NumOfSensors

				if (!vl[n].checkID()) connected_t[(NumOfZonesPerSensor * n) + z] = false;
				if (connected_t[(NumOfZonesPerSensor * n) + z])
				{
					vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
					vl[n].startRanging();
				}
			}
#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
            if (z == 0) { //Gestion collision
                for (int c = 0; c < NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        while (!vl_collision[c].checkForDataReady()) {
                            threads.yield();
                        }
                        vl_collision[c].getResult(&res_collision[c]);
                        vl_collision[c].clearInterrupt();
                        vl_collision[c].stopRanging();

                        filteredResult_coll[c] = res_collision[c].Distance;
                        distance_coll[c] = res_collision[c].Distance;
                        status_coll[c] = res_collision[c].Status;
                        NumSPADs_coll[c] = res_collision[c].NumSPADs;
                        SigPerSPAD_coll[c] = res_collision[c].SigPerSPAD;
                        Ambient_coll[c] = res_collision[c].Ambient;
                    }
                }
            }
#endif

			//tofVLReadyForCalculation = false;
			threads.yield();

			for (int n = 0; n < NumOfSensors / 2; n++)
			{ //NumOfSensors

				if (connected_t[(NumOfZonesPerSensor * n) + z])
				{

					while (!vl[n].checkForDataReady())
					{
						threads.yield();
					}

					//vl[n].setOffset(OffsetCal[(NumOfZonesPerSensor * n) + z]);
					//vl[n].setXTalk(OffsetCalxTalk[(NumOfZonesPerSensor * n) + z]);
					vl[n].getResult(&res[(NumOfZonesPerSensor * n) + z]);
					vl[n].clearInterrupt();
					vl[n].stopRanging();

					//(3 - z) c'est pour inverser les capteurs et lire les zones dans l'autre sens
					if (((res[(NumOfZonesPerSensor * n) + z].NumSPADs < 3
							&& res[(NumOfZonesPerSensor * n) + z].Distance < 150
							&& res[(NumOfZonesPerSensor * n) + z].Status == RangeValid)
							||

							(res[(NumOfZonesPerSensor * n) + z].NumSPADs < 12
									&& res[(NumOfZonesPerSensor * n) + z].Distance > 150
									&& (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid
											|| res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
									&& res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 600 //600
									&& res[(NumOfZonesPerSensor * n) + z].Ambient < 3100)) //< 1100    ; 500 25000 si on se fait eblouir

							|| (((res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit) //2ème cas
							&& res[(NumOfZonesPerSensor * n) + z].Distance > 1300
									&& res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 3000
									&& res[(NumOfZonesPerSensor * n) + z].Ambient < 15000)))
					{

						filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] =
								res[(NumOfZonesPerSensor * n) + z].Distance;
					} else
					{
						filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					}

					//save other data with inversion of zone
					distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
					status_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Status;
					NumSPADs_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].NumSPADs;
					SigPerSPAD_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].SigPerSPAD;
					Ambient_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Ambient;
				} else
				{
					//ERROR
					filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					status_t[(NumOfZonesPerSensor * n) + (3 - z)] = 5;
				}
			}

		}

//		Serial.print(" =>unblock ");
//		tofVLunblock = true;

		shared_endloop1 = 1;
		threads.yield();

		/*
		 uint timeout = 0;
		 //on attend tant que le 2e thread n'a pas fini pour mettre à jour que les données;
		 while (!shared_endloop2)
		 {
		 threads.yield();
		 //Serial.println(" 1.waitendloop2 ");
		 timeout++;
		 if(timeout>1000)
		 {

		 tofVLunblock = true;
		 Serial.print(" =>unblock ");
		 }
		 }



		 //algo de correction primaire

		 //1. Cas du zero sur la distance qui arrive de temps en temps, on utilise l'ancienne donnée si elle existe
		 for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
		 {
		 if (filteredResult[n] == 0)
		 {
		 if (filteredResultPrevious[n] != 0) filteredResult[n] = filteredResultPrevious[n];
		 }
		 }

		 //test du 0  0 tout tout pres
		 for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
		 {
		 if (filteredResult[n] == 0)
		 {
		 filteredResult[n] = 30;
		 }
		 }

		 //todo FAIRE LA TRANSITION INF A 3 ET MAX -3

		 //3. sur plusieurs valeurs, si SigPerSPAD_t < 5, on ne garde que ces valeurs <5
		 for (int n = 3; n < (NumOfSensors * NumOfZonesPerSensor) - 3; n++)
		 {
		 if ((filteredResult[n] > (filteredResult[n - 3] - 50)) & (filteredResult[n] < (filteredResult[n - 3] + 50)))
		 {
		 if ((status_t[n - 3] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {
		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n - 3] = 1;
		 }

		 }
		 }
		 if ((filteredResult[n] > (filteredResult[n - 2] - 50)) & (filteredResult[n] < (filteredResult[n - 2] + 50)))
		 {
		 if ((status_t[n - 2] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {
		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n - 2] = 1;
		 }

		 }
		 }
		 if ((filteredResult[n] > (filteredResult[n - 1] - 50)) & (filteredResult[n] < (filteredResult[n - 1] + 50)))
		 {
		 if ((status_t[n - 1] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {

		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n - 1] = 1;
		 }

		 }
		 }
		 if ((filteredResult[n] > (filteredResult[n + 1] - 50)) & (filteredResult[n] < (filteredResult[n + 1] + 50)))
		 {
		 if ((status_t[n + 1] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {
		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n + 1] = 1;
		 }

		 }
		 }
		 if ((filteredResult[n] > (filteredResult[n + 2] - 50)) & (filteredResult[n] < (filteredResult[n + 2] + 50)))
		 {
		 if ((status_t[n + 2] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {
		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n + 2] = 1;
		 }
		 }

		 }
		 if ((filteredResult[n] > (filteredResult[n + 3] - 50)) & (filteredResult[n] < (filteredResult[n + 3] + 50)))
		 {
		 if ((status_t[n + 3] == PhaseOutOfLimit)
		 & (((status_t[n] == RangeValid) & (NumSPADs_t[n] < 13)) | (NumSPADs_t[n] < 5)))
		 {
		 if (SigPerSPAD_t[n] > 14000)
		 {
		 filteredResult[n + 3] = 1;
		 }
		 }

		 }
		 }
		 //3.greenHandDistance
		 for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
		 {
		 if ((status_t[n] == RangeValid) & (SigPerSPAD_t[n] > 1000) & (NumSPADs_t[n] > 10)) //| (status_t[n] == SignalFail)| (status_t[n] == PhaseOutOfLimit))
		 {

		 greenHandDistance[n] = distance_t[n];

		 } else
		 {
		 greenHandDistance[n] = 0;
		 }
		 }

		 //4.sauvegarde de l'ancienne valeur
		 for (int n = 0; n < NumOfSensors * NumOfZonesPerSensor; n++)
		 {
		 if (filteredResult[n] >= 0) filteredResultPrevious[n] = distance_t[n];
		 }

		 //Les DONNÉES sont PRETES : on copie les données (avec semaphore)

		 filteredResultWorkingCopy_mutex.lock();
		 //filteredResultWorkingCopy = filteredResult;
		 memcpy(&filteredResultWorkingCopy, &filteredResult, sizeof(filteredResult));
		 filteredResultWorkingCopy_mutex.unlock();

		 tofVLReadyForCalculation = true;
		 */

		//threads.yield();
		//threads.delay(2);
//		if (ontime == 1)
//		{
//			threads.delay(50);
//			ontime = 0;
//		}
	}
}

void loopvl2()
{
	//int ontime = 1;
	shared_endloop2 = 1;

	while (1)
	{

		//Serial.println("loopvl2");

		//on attend que le 1er thread demarre
		while (shared_endloop2)
		{
			threads.yield();
			//Serial.println(" 2.shared_endloop1 ");
		}
//		tofVLunblock=false;
//		shared_endloop2 = 0;
//		tofVLReadyForCalculation = false;
		//threads.yield();

		for (int z = 0; z < NumOfZonesPerSensor; z++)
		{

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
            if (z == 0) { //Gestion collision
                for (int c = NumOfCollisionSensors; c < NumOfCollisionSensors + NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        vl_collision[c].setROI(16, 16, 199);
                        vl_collision[c].startRanging();
                    }
                }
            }
#endif

			for (int n = (NumOfSensors / 2); n < NumOfSensors; n++)
			{ //NumOfSensors

				if (!vl[n].checkID()) connected_t[(NumOfZonesPerSensor * n) + z] = false;
				if (connected_t[(NumOfZonesPerSensor * n) + z])
				{
					vl[n].setROI(WidthOfSPADsPerZone, 8, center[NumOfSPADsShiftPerZone * z + NumOfSPADsToStartZone]);
					vl[n].startRanging();
				}
			}

#ifdef SENSORS_VL_CLOSED_COLLISION_ACTIVATED
            if (z == 0) //Gestion collision
                    {
                for (int c = NumOfCollisionSensors; c < NumOfCollisionSensors + NumOfCollisionSensors; c++) {
                    if (connected_coll[c]) {
                        while (!vl_collision[c].checkForDataReady()) {
                            threads.yield();
                        }
                        vl_collision[c].getResult(&res_collision[c]);
                        vl_collision[c].clearInterrupt();
                        vl_collision[c].stopRanging();

                        filteredResult_coll[c] = res_collision[c].Distance;
                        distance_coll[c] = res_collision[c].Distance;
                        status_coll[c] = res_collision[c].Status;
                        NumSPADs_coll[c] = res_collision[c].NumSPADs;
                        SigPerSPAD_coll[c] = res_collision[c].SigPerSPAD;
                        Ambient_coll[c] = res_collision[c].Ambient;
                    }
                }
            }
#endif

//			tofVLReadyForCalculation = false;
			//threads.yield();

			for (int n = (NumOfSensors / 2); n < NumOfSensors; n++)
			{ //NumOfSensors

				if (connected_t[(NumOfZonesPerSensor * n) + z])
				{
					while (!vl[n].checkForDataReady()) // || !tofVLunblock)
					{
						threads.yield();
						//
//						Serial.print(" 2.waitdata");
//						Serial.println(tofVLunblock);
					}
//					if (!tofVLunblock)
//					{
					//vl[n].setOffset(OffsetCal[(NumOfZonesPerSensor * n) + z]);
					//vl[n].setXTalk(OffsetCalxTalk[(NumOfZonesPerSensor * n) + z]);
					vl[n].getResult(&res[(NumOfZonesPerSensor * n) + z]);

					vl[n].clearInterrupt();
					vl[n].stopRanging();

					if (((res[(NumOfZonesPerSensor * n) + z].NumSPADs < 2
							&& res[(NumOfZonesPerSensor * n) + z].Distance < 150
							&& res[(NumOfZonesPerSensor * n) + z].Status == RangeValid)
							||

							(res[(NumOfZonesPerSensor * n) + z].NumSPADs < 12
									&& res[(NumOfZonesPerSensor * n) + z].Distance > 150
									&& (res[(NumOfZonesPerSensor * n) + z].Status == RangeValid
											|| res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit)
									&& res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 600 //600
									&& res[(NumOfZonesPerSensor * n) + z].Ambient < 3100)) //< 1100    ; 500 25000 si on se fait eblouir

							|| (((res[(NumOfZonesPerSensor * n) + z].Status == PhaseOutOfLimit) //2ème cas loin
							&& res[(NumOfZonesPerSensor * n) + z].Distance > 1300
									&& res[(NumOfZonesPerSensor * n) + z].SigPerSPAD > 3000
									&& res[(NumOfZonesPerSensor * n) + z].Ambient < 15000)))
					{

						filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] =
								res[(NumOfZonesPerSensor * n) + z].Distance;
					} else
					{
						filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					}

					//save other data with inversion of zone
					distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Distance;
					status_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Status;
					NumSPADs_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].NumSPADs;
					SigPerSPAD_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].SigPerSPAD;
					Ambient_t[(NumOfZonesPerSensor * n) + (3 - z)] = res[(NumOfZonesPerSensor * n) + z].Ambient;

				} else
				{
					//ERROR
					filteredResult[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					distance_t[(NumOfZonesPerSensor * n) + (3 - z)] = 1;
					status_t[(NumOfZonesPerSensor * n) + (3 - z)] = 5;
				}
//				}
			}
		}

		shared_endloop2 = 1;
		threads.yield();
		//threads.delay(8);
//		if (ontime == 1)
//		{
//			//threads.delay(60);
//			ontime = 0;
//		}

	}
}

