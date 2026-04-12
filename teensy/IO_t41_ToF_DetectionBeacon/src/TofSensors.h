/**
 * @file TofSensors.h
 * @brief Detection 360° des robots adverses par capteurs VL53L1X Time-of-Flight.
 *
 * Ce module gere 18 capteurs VL53L1X disposes en couronne sur la balise beacon.
 * Chaque capteur scanne 4 zones SPAD, soit 72 zones sur 360° (~5° par zone).
 * Les capteurs sont repartis sur 2 bus I2C :
 *   - Front (9 capteurs) : Wire1 (SDA1/SCL1)
 *   - Back  (9 capteurs) : Wire  (SDA/SCL)
 *
 * Les donnees de detection (positions, angles, distances) sont exposees
 * au cerveau OPOS6UL via une interface I2C esclave (adresse 0x2D).
 *
 * @author cho (PM-ROBOTIX)
 * @date Jan 25, 2021
 */

#ifndef TOFSENSORS_H_
#define TOFSENSORS_H_

#include "SparkFun_VL53L1X.h"
#include "TeensyThreads.h"

//#define ACTIVATE_SENSORS_VL_360_DETECTION 1 //TODO a faire

/** @brief Timing budget de chaque mesure VL53L1X en millisecondes. */
#define TIMING_UDGET_IN_MS 15

/**
 * @brief Registres I2C du slave beacon (adresse 0x2D).
 *
 * Ces parametres sont organises en 2 blocs contigus par sens de communication :
 * - Bloc 1 (reg 0-4) : OPOS6UL -> Teensy (config, affichage, etat match)
 * - Bloc 2 (reg 5-8) : Teensy (LCD tactile) -> OPOS6UL (choix operateur pre-match)
 *
 * Regle d'atomicite : un seul ecrivain par byte. Les acces uint8_t/int8_t
 * sur Cortex-M7 etant atomiques, aucun mutex n'est necessaire sur ces champs.
 * Exception : ledLuminosity (reg 1) peut etre ecrit par le LCD en phase prepa
 * ET par l'OPOS6UL en phase match (dernier ecrivain gagne, pas de conflit
 * en pratique car les phases sont sequentielles).
 *
 * Voir teensy/IO_t41_ToF_DetectionBeacon/ARCHITECTURE_BEACON.md
 * section "Menu pre-match (LCD tactile)" pour l'architecture complete.
 */
struct Settings {
	// === Bloc 1 : OPOS6UL -> Teensy (5 bytes) ===
	int8_t  numOfBots     = 3;   ///< Reg 0. Nb max d'adv a detecter (W: OPOS6UL).
	int8_t  ledLuminosity = 50;  ///< Reg 1. Luminosite LED matrix 0/50/100 (W: OPOS6UL).
	uint8_t matchPoints   = 0;   ///< Reg 2. Score match sur LED matrix + LCD (W: OPOS6UL).
	uint8_t matchState    = 0;   ///< Reg 3. Etat match: 0=prepa, 1=en cours, 2=fini (W: OPOS6UL).
	uint8_t lcdBacklight  = 1;   ///< Reg 4. Backlight LCD: 0=off, 1=on (W: OPOS6UL).

	// === Bloc 2 : Teensy (LCD) -> OPOS6UL (4 bytes) ===
	uint8_t matchColor    = 0;   ///< Reg 5. Couleur equipe: 0=bleu, 1=jaune (W: LCD).
	uint8_t strategy      = 0;   ///< Reg 6. N° strategie IA 1..3 (W: LCD).
	uint8_t testMode      = 0;   ///< Reg 7. Test materiel: 0=aucun, 1..5=test dedie (W: LCD).
	uint8_t advDiameter   = 40;  ///< Reg 8. Diametre adversaire en cm, defaut 40 (W: LCD).
};
static_assert(sizeof(Settings) == 9, "Settings must be exactly 9 bytes for I2C layout");

/**
 * @brief Registres I2C en lecture seule pour le master (OPOS6UL).
 *
 * Contiennent les resultats de detection : positions (x, y, angle),
 * distances, donnees brutes par zone et flags de status.
 *
 * @note Les registres sont mis a jour a chaque cycle d'acquisition
 *       et proteges par un mutex (registers_new_data_lock).
 */
struct Registers {
	uint8_t flags = 0x80;        ///< Reg 4. bit0: new data available, bit7: alive.
	//TODO FLAGS
	//overflow more than 3 beacons
	//more than 6 contigus
	//default config settings is changed by master or not. in case of reset, the master can know and reconfigure

	uint8_t nbDetectedBots = 0;  ///< Reg 5. Nombre de robots adverses detectes (0-4).

	// --- Distances collision (capteurs rapproches, optionnels) ---
	int16_t c1_mm = 0;        ///< Reg 6. Distance collision capteur 1 (mm).
	int16_t c2_mm = 0;        ///< Reg 8. Distance collision capteur 2 (mm).
	int16_t c3_mm = 0;        ///< Reg 10. Distance collision capteur 3 (mm).
	int16_t c4_mm = 0;        ///< Reg 12. Distance collision capteur 4 (mm).
	int16_t c5_mm = 0;        ///< Reg 14. Distance collision capteur 5 (mm).
	int16_t c6_mm = 0;        ///< Reg 16. Distance collision capteur 6 (mm).
	int16_t c7_mm = 0;        ///< Reg 18. Distance collision capteur 7 (mm).
	int16_t c8_mm = 0;        ///< Reg 20. Distance collision capteur 8 (mm).

	int16_t reserved = 0;     ///< Reg 22. Reserve.

	// --- Position robot adverse 1 ---
	int16_t x1_mm = 0;        ///< Reg 24. Coordonnee X robot 1 (mm).
	int16_t y1_mm = 0;        ///< Reg 26. Coordonnee Y robot 1 (mm).
	float a1_deg = 0.0;       ///< Reg 28. Angle robot 1 (degres).

	// --- Position robot adverse 2 ---
	int16_t x2_mm = 0;        ///< Reg 32. Coordonnee X robot 2 (mm).
	int16_t y2_mm = 0;        ///< Reg 34. Coordonnee Y robot 2 (mm).
	float a2_deg = 0.0;       ///< Reg 36. Angle robot 2 (degres).

	// --- Position robot adverse 3 ---
	int16_t x3_mm = 0;        ///< Reg 40. Coordonnee X robot 3 (mm).
	int16_t y3_mm = 0;        ///< Reg 42. Coordonnee Y robot 3 (mm).
	float a3_deg = 0.0;       ///< Reg 44. Angle robot 3 (degres).

	// --- Position robot adverse 4 ---
	int16_t x4_mm = 0;        ///< Reg 48. Coordonnee X robot 4 (mm).
	int16_t y4_mm = 0;        ///< Reg 50. Coordonnee Y robot 4 (mm).
	float a4_deg = 0.0;       ///< Reg 52. Angle robot 4 (degres).

	// --- Distances centre-a-centre ---
	int16_t d1_mm = 0;        ///< Reg 56. Distance centre-a-centre robot 1 (mm).
	int16_t d2_mm = 0;        ///< Reg 58. Distance centre-a-centre robot 2 (mm).
	int16_t d3_mm = 0;        ///< Reg 60. Distance centre-a-centre robot 3 (mm).
	int16_t d4_mm = 0;        ///< Reg 62. Distance centre-a-centre robot 4 (mm).

	// --- Donnees brutes par zone pour chaque robot detecte ---
	// Pour chaque robot zN : position de debut, nombre de zones, et distances par zone.

	uint8_t z1_p = 0;          ///< Reg 64. Position de la premiere zone du robot 1 (0-71).
	uint8_t z1_n = 0;          ///< Reg 65. Nombre de zones contigues detectees pour le robot 1 (1-7).
	uint16_t z1_1 = 0;         ///< Reg 66. Distance zone 1 du robot 1 (mm).
	uint16_t z1_2 = 0;         ///< Reg 68. Distance zone 2 du robot 1 (mm).
	uint16_t z1_3 = 0;         ///< Reg 70. Distance zone 3 du robot 1 (mm).
	uint16_t z1_4 = 0;         ///< Reg 72. Distance zone 4 du robot 1 (mm).
	uint16_t z1_5 = 0;         ///< Reg 74. Distance zone 5 du robot 1 (mm).
	uint16_t z1_6 = 0;         ///< Reg 76. Distance zone 6 du robot 1 (mm).
	uint16_t z1_7 = 0;         ///< Reg 78. Distance zone 7 du robot 1 (mm).

	uint8_t z2_p = 0;          ///< Reg 80. Position de la premiere zone du robot 2 (0-71).
	uint8_t z2_n = 0;          ///< Reg 81. Nombre de zones contigues detectees pour le robot 2 (1-7).
	uint16_t z2_1 = 0;         ///< Reg 82. Distance zone 1 du robot 2 (mm).
	uint16_t z2_2 = 0;         ///< Reg 84. Distance zone 2 du robot 2 (mm).
	uint16_t z2_3 = 0;         ///< Reg 86. Distance zone 3 du robot 2 (mm).
	uint16_t z2_4 = 0;         ///< Reg 88. Distance zone 4 du robot 2 (mm).
	uint16_t z2_5 = 0;         ///< Reg 90. Distance zone 5 du robot 2 (mm).
	uint16_t z2_6 = 0;         ///< Reg 92. Distance zone 6 du robot 2 (mm).
	uint16_t z2_7 = 0;         ///< Reg 94. Distance zone 7 du robot 2 (mm).

	uint8_t z3_p = 0;          ///< Reg 96. Position de la premiere zone du robot 3 (0-71).
	uint8_t z3_n = 0;          ///< Reg 97. Nombre de zones contigues detectees pour le robot 3 (1-7).
	uint16_t z3_1 = 0;         ///< Reg 98. Distance zone 1 du robot 3 (mm).
	uint16_t z3_2 = 0;         ///< Reg 100. Distance zone 2 du robot 3 (mm).
	uint16_t z3_3 = 0;         ///< Reg 102. Distance zone 3 du robot 3 (mm).
	uint16_t z3_4 = 0;         ///< Reg 104. Distance zone 4 du robot 3 (mm).
	uint16_t z3_5 = 0;         ///< Reg 106. Distance zone 5 du robot 3 (mm).
	uint16_t z3_6 = 0;         ///< Reg 108. Distance zone 6 du robot 3 (mm).
	uint16_t z3_7 = 0;         ///< Reg 110. Distance zone 7 du robot 3 (mm).

	uint8_t z4_p = 0;          ///< Reg 112. Position de la premiere zone du robot 4 (0-71).
	uint8_t z4_n = 0;          ///< Reg 113. Nombre de zones contigues detectees pour le robot 4 (1-7).
	uint16_t z4_1 = 0;         ///< Reg 114. Distance zone 1 du robot 4 (mm).
	uint16_t z4_2 = 0;         ///< Reg 116. Distance zone 2 du robot 4 (mm).
	uint16_t z4_3 = 0;         ///< Reg 118. Distance zone 3 du robot 4 (mm).
	uint16_t z4_4 = 0;         ///< Reg 120. Distance zone 4 du robot 4 (mm).
	uint16_t z4_5 = 0;         ///< Reg 122. Distance zone 5 du robot 4 (mm).
	uint16_t z4_6 = 0;         ///< Reg 124. Distance zone 6 du robot 4 (mm).
	uint16_t z4_7 = 0;         ///< Reg 126. Distance zone 7 du robot 4 (mm).

	// --- Timing de mesure par robot (pour synchronisation OPOS6UL) ---
	uint16_t t1_us = 0;        ///< Reg 128. Delta moyen mesure robot 1 (us depuis debut cycle).
	uint16_t t2_us = 0;        ///< Reg 130. Delta moyen mesure robot 2.
	uint16_t t3_us = 0;        ///< Reg 132. Delta moyen mesure robot 3.
	uint16_t t4_us = 0;        ///< Reg 134. Delta moyen mesure robot 4.
	uint32_t seq = 0;          ///< Reg 136. Numero de sequence (incremente chaque cycle).
};

/**
 * @brief Scanne un bus I2C et affiche les adresses des peripheriques trouves.
 * @param w Bus I2C a scanner (Wire ou Wire1).
 * @return Nombre de peripheriques trouves.
 */
int scani2c(TwoWire w);

/**
 * @brief Initialise les 18 capteurs VL53L1X et demarre les threads d'acquisition.
 *
 * Sequence d'initialisation :
 * 1. Configuration de l'esclave I2C (adresse 0x2D)
 * 2. Activation et re-adressage individuel de chaque capteur (0x15 a 0x26)
 * 3. Configuration mode Short, timing budget, periode inter-mesures
 * 4. Scan I2C de verification
 * 5. Lancement des threads loopvl1 (Front) et loopvl2 (Back)
 */
void tof_setup();

/**
 * @brief Boucle principale de traitement ToF (appelee depuis loop()).
 *
 * Synchronise les threads d'acquisition, applique les filtres de correction,
 * calcule les positions des robots adverses et met a jour les registres I2C.
 *
 * @param debug Si != 0, affiche les donnees detaillees sur le port serie
 *              (distances, status, NumSPADs, SigPerSPAD, Ambient).
 */
void tof_loop(int debug = 0);

/**
 * @brief Thread d'acquisition des 9 capteurs Front (Wire1).
 *
 * Boucle infinie qui, pour chaque zone SPAD (0-3) de chaque capteur (0-8) :
 * 1. Configure le ROI (Region of Interest) du capteur
 * 2. Demarre la mesure (startRanging)
 * 3. Attend les donnees avec timeout (TIMING_UDGET_IN_MS * 3)
 * 4. Filtre les resultats selon NumSPADs, SigPerSPAD, Ambient et Status
 * 5. Stocke dans filteredResult[] avec inversion de zone (3-z)
 */
void loopvl1();

/**
 * @brief Thread d'acquisition des 9 capteurs Back (Wire).
 * @see loopvl1() — meme logique pour les capteurs 9 a 17.
 */
void loopvl2();

/**
 * @brief Calcule la position (angle, distance, coordonnees x/y) des robots detectes.
 *
 * Algorithme :
 * 1. Identifie les groupes de zones contigues dans filteredResult (= 1 robot)
 * 2. Gere le chevauchement circulaire (zone 71 → zone 0)
 * 3. Calcule l'angle moyen de chaque groupe
 * 4. Calcule la distance moyenne (avec rejet du minimum pour robustesse)
 * 5. Convertit en coordonnees cartesiennes (x, y) par trigonometrie
 *
 * @param decalage_deg Offset angulaire de calibration (degres).
 * @param new_values   Registres de sortie a remplir.
 * @param filteredResult Tableau des distances filtrees par zone (72 valeurs).
 * @return Nombre de robots detectes (0-4).
 */
int8_t calculPosition(float decalage_deg, Registers &new_values, uint16_t *filteredResult);

/**
 * @brief Callback ISR appele apres lecture I2C par le master.
 *
 * Remet a zero le bit "new data" (bit0 de flags) pour signaler
 * au master que les donnees ont ete lues.
 *
 * @param reg_num Numero du registre lu.
 */
void on_read_isr(uint8_t reg_num);

#endif /* TOFSENSORS_H_ */
