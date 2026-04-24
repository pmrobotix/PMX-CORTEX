/*!
 * \file
 * \brief Communication I2C avec la balise Teensy (BeaconSensors).
 *
 * Lit les registres I2C de la Teensy (0x2D) : positions adversaires,
 * distances collision (c1-c8), timing de mesure (t1-t4_us, seq).
 *
 * Utilise AsI2cAtomic (repeated start) pour eviter la desynchronisation
 * du toggle got_reg_num du slave I2CRegisterSlave cote Teensy.
 */

#ifndef DRIVER_OPOS6UL_ARM_BEACONSENSORS_HPP_
#define DRIVER_OPOS6UL_ARM_BEACONSENSORS_HPP_

#include "AsI2cAtomic.hpp"
#include "log/LoggerFactory.hpp"

//#define ADDRESS_BeaconSensors   0x2D
// Offset I2C des Registers = sizeof(Settings) sur la Teensy.
// Settings fait maintenant 19 bytes (ajout 8 pickup_Pn en bloc 2 pour la config
// des zones de prise pre-match, voir ARCHITECTURE_BEACON.md et MATCH_CONFIG_UI.md).
// IMPORTANT: doit matcher TofSensors.h cote Teensy, sinon ToF detection cassee.
#define SETTINGS_SIZE_BeaconSensors 19
#define DATA_BeaconSensors      SETTINGS_SIZE_BeaconSensors //adresse des donnees Registers a recuperer
#define NUMOFBOTS_BeaconSensors 0x00

//WEBSITE REFERENCE : convert float to byte array  source: http://mbed.org/forum/helloworld/topic/2053/

union float2bytes_t // union consists of one variable represented in a number of different ways
{
    float f;
    unsigned char bytes[sizeof(float)];
};

// Registres I2C du slave beacon (adresse 0x2D).
// Miroir de la struct Settings cote Teensy (TofSensors.h).
// Voir ARCHITECTURE_BEACON.md section "Menu pre-match (LCD tactile)".
struct Settings
{
    // === Bloc 1 : OPOS6UL -> Teensy (5 bytes) ===
    int8_t  numOfBots     = 3;   // Reg 0. Nb max adv a detecter (W: OPOS6UL).
    int8_t  ledLuminosity = 10; // Reg 1. Luminosite LED matrix 0..100 (W: OPOS6UL au boot pour le match).
    uint8_t matchPoints   = 0;   // Reg 2. Score match sur LED matrix + LCD (W: OPOS6UL).
    uint8_t matchState    = 0;   // Reg 3. 0=prepa, 1=match, 2=fini (W: OPOS6UL).
    uint8_t lcdBacklight  = 1;   // Reg 4. 0=off, 1=on (W: OPOS6UL).

    // === Bloc 2 : Teensy (LCD) -> OPOS6UL (5 bytes) ===
    uint8_t matchColor    = 0;   // Reg 5. Couleur equipe: 0=bleu, 1=jaune (W: LCD).
    uint8_t strategy      = 0;   // Reg 6. N° strategie IA 1..3 (W: LCD).
    uint8_t testMode      = 0;   // Reg 7. Test materiel: 0=aucun, 1..5 (W: LCD).
    uint8_t advDiameter   = 40;  // Reg 8. Diametre adversaire en cm (W: LCD).
    uint8_t actionReq     = 0;   // Reg 9. 1 = bouton SETPOS/RESET clique (sens selon matchState).
                                 // OPOS6UL remet a 0 apres consommation.

    // Zones de prise (config pre-match). Index 0..5 dans cycle canonique :
    // 0=BBYY, 1=YYBB, 2=BYYB, 3=YBBY, 4=BYBY, 5=YBYB. Defaut 0=BBYY.
    // Voir teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md.
    uint8_t pickup_P1     = 0;   // Reg 10. P1  (V, bleu)   (W: LCD)
    uint8_t pickup_P2     = 0;   // Reg 11. P2  (V, bleu)   (W: LCD)
    uint8_t pickup_P3     = 0;   // Reg 12. P3  (H, bleu)   (W: LCD)
    uint8_t pickup_P4     = 0;   // Reg 13. P4  (H, bleu)   (W: LCD)
    uint8_t pickup_P11    = 0;   // Reg 14. P11 (V, jaune)  (W: LCD)
    uint8_t pickup_P12    = 0;   // Reg 15. P12 (V, jaune)  (W: LCD)
    uint8_t pickup_P13    = 0;   // Reg 16. P13 (H, jaune)  (W: LCD)
    uint8_t pickup_P14    = 0;   // Reg 17. P14 (H, jaune)  (W: LCD)

    // === Bloc 3 : compteur de clics touch (1 byte) ===
    uint8_t seq_touch     = 0;   // Reg 18. Incremente par Teensy a chaque clic LVGL.
};

// Registers that the caller can only read
struct Registers
{
    uint8_t flags = 0;        // Register 4. bit 0 => new data available
    uint8_t nbDetectedBots = 0; //Register 5.Nombre de balises détectées.
    int16_t c1_mm = 0;        // Register 6. //AV GAUCHE BAS
    int16_t c2_mm = 0;        // Register 8. //AV GAUCHE HAUT
    int16_t c3_mm = 0;        // Register 10. //AV DROIT BAS
    int16_t c4_mm = 0;        // Register 12. //AV DROIT HAUT
    int16_t c5_mm = 0;        // Register 14. //AR GAUCHE BAS
    int16_t c6_mm = 0;        // Register 16. //AR GAUCHE HAUT
    int16_t c7_mm = 0;        // Register 18. //AR DROIT BAS
    int16_t c8_mm = 0;        // Register 20. //AR DROIT HAUT

    int16_t reserved = 0;        // Register 22.

    int16_t x1_mm = 0;        // Register 24.
    int16_t y1_mm = 0;        // Register 26.
    float a1_deg = 0.0;       // Register 28.

    int16_t x2_mm = 0;        // Register 32.
    int16_t y2_mm = 0;        // Register 34.
    float a2_deg = 0.0;       // Register 36.

    int16_t x3_mm = 0;        // Register 40.
    int16_t y3_mm = 0;        // Register 42.
    float a3_deg = 0.0;       // Register 44.

    int16_t x4_mm = 0;        // Register 48
    int16_t y4_mm = 0;        // Register 50
    float a4_deg = 0.0;       // Register 52

    int16_t d1_mm = 0;        // Register 56.    centre à centre
    int16_t d2_mm = 0;        // Register 58.
    int16_t d3_mm = 0;        // Register 60.
    int16_t d4_mm = 0;        // Register 62.

//TODO decaler de 2
    int8_t z1_p = 0;          // Register 62. position de la zone z1_1 (entre 0 et 71).
    int8_t z1_n = 0; // Register 63. nombre de zones detectées pour la balise z1 (entre 1 et 7) afin d'economiser les données.
    int16_t z1_1 = 0;         // Register 64.
    int16_t z1_2 = 0;         // Register 66.
    int16_t z1_3 = 0;         // Register 68.
    int16_t z1_4 = 0;         // Register 70.
    int16_t z1_5 = 0;         // Register 72.
    int16_t z1_6 = 0;         // Register 74.
    int16_t z1_7 = 0;         // Register 76.

    int8_t z2_p = 0;          // Register 78.position de la zone z2_1 (entre 0 et 71).
    int8_t z2_n = 0; // Register 79.nombre de zones detectées pour la balise z2 (entre 1 et 7) afin d'economiser les données.
    int16_t z2_1 = 0;         // Register 80.
    int16_t z2_2 = 0;         // Register 82.
    int16_t z2_3 = 0;         // Register 84.
    int16_t z2_4 = 0;         // Register 86.
    int16_t z2_5 = 0;         // Register 88.
    int16_t z2_6 = 0;         // Register 90.
    int16_t z2_7 = 0;         // Register 92.

    int8_t z3_p = 0;          // Register 94.position de la zone z3_1 (entre 0 et 71).
    int8_t z3_n = 0; // Register 95.nombre de zones detectées pour la balise z3 (entre 1 et 7) afin d'economiser les données.
    int16_t z3_1 = 0;         // Register 96.
    int16_t z3_2 = 0;         // Register 98.
    int16_t z3_3 = 0;         // Register 100.
    int16_t z3_4 = 0;         // Register 102.
    int16_t z3_5 = 0;         // Register 104.
    int16_t z3_6 = 0;         // Register 106.
    int16_t z3_7 = 0;         // Register 108.

    int8_t z4_p = 0;          // Register 110.position de la zone z4_1 (entre 0 et 71).
    int8_t z4_n = 0; // Register 111.nombre de zones detectées pour la balise z4 (entre 1 et 7) afin d'economiser les données.
    int16_t z4_1 = 0;         // Register 112.
    int16_t z4_2 = 0;         // Register 114.
    int16_t z4_3 = 0;         // Register 116.
    int16_t z4_4 = 0;         // Register 118.
    int16_t z4_5 = 0;         // Register 120.
    int16_t z4_6 = 0;         // Register 122.
    int16_t z4_7 = 0;         // Register 124.

    // --- Timing de mesure (pour synchronisation OPOS6UL) ---
    uint16_t t1_us = 0;      // Register 124. Delta moyen mesure robot 1 (us depuis debut cycle).
    uint16_t t2_us = 0;      // Register 126. Delta moyen mesure robot 2.
    uint16_t t3_us = 0;      // Register 128. Delta moyen mesure robot 3.
    uint16_t t4_us = 0;      // Register 130. Delta moyen mesure robot 4.
    uint32_t seq = 0;         // Register 132. Numero de sequence (incremente chaque cycle).

};

class BeaconSensors
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref BeaconSensors(OPOS6UL).
     */
    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("BeaconSensors.OPO");
        return instance;
    }

    AsI2cAtomic i2c_;  ///< Bus I2C unique (repeated start pour lectures, fd unique).
    bool connected_BeaconSensors_;

public:

    /*!
     * \brief Constructor.
     */
    BeaconSensors(int bus, unsigned char address);

    /*!
     * \brief Destructor.
     */
    ~BeaconSensors()
    {
    }
    /*!
     *  \brief initialise i2c
     *
     */
    bool begin(Settings settings);

    /*!
     *  \brief Lit le registre flags (1 octet) pour verifier si de nouvelles donnees sont disponibles.
     *  \return flags (bit0 = new data, bit7 = alive, 0xFF = erreur I2C)
     */
    uint8_t readFlag();

    /*!
     *  \brief Lit tous les registres de la balise.
     */
    /*!
     *  \brief DEPRECATED — utiliser getDataFull() a la place.
     *  Lit les registres en 7 transactions I2C separees (fragile).
     */
    [[deprecated("Utiliser getDataFull() (1 seule transaction I2C)")]]
    Registers getData();

    /*!
     *  \brief Lit tous les registres en une seule transaction I2C (136 bytes).
     *  Plus robuste que getData() (1 transaction au lieu de 7).
     */
    Registers getDataFull();

    /*!
     * \brief Lit le bloc Settings complet (registres 0..18, 19 bytes avec
     *        actionReq + 8 pickup_Pn + seq_touch).
     * \param out Struct a remplir.
     * \return true si lecture OK, false si erreur I2C.
     */
    bool readSettings(struct Settings &out);

    void display(int number);

    void writeLedLuminosity(uint8_t lum);

    /*!
     * \brief Ecriture par champ du bloc Settings. Retourne true si OK.
     * Voir ARCHITECTURE_BEACON.md pour le mapping reg.
     */
    bool writeNumOfBots(int8_t n);     // reg 0
    bool writeMatchPoints(uint8_t p);  // reg 2
    bool writeMatchState(uint8_t s);   // reg 3
    bool writeLcdBacklight(uint8_t b); // reg 4
    bool writeMatchColor(uint8_t c);   // reg 5
    bool writeStrategy(uint8_t s);     // reg 6
    bool writeAdvDiameter(uint8_t d);  // reg 8
    bool writeActionReq(uint8_t v);    // reg 9 (used by OPOS6UL to clear after consuming)

    /*!
     * \brief Retourne true si begin() a reussi et que la balise repond.
     */
    bool is_connected() const { return connected_BeaconSensors_; }

    int ScanBus();
};

#endif
