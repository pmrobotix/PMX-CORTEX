#ifndef OPOS6UL_SENSORSDRIVER_HPP_
#define OPOS6UL_SENSORSDRIVER_HPP_

#include "interface/ASensorsDriver.hpp"
#include "utils/PointerList.hpp"
#include "log/LoggerFactory.hpp"
#include "BeaconSensors.hpp"
#include "Gp2y0e02b.hpp"

#define ADDRESS_BeaconSensors   0x2D


#define ADDRESS_gp2y0e02b       0x40 //0x80 >> 1  // Arduino uses 7 bit addressing so we shift address right one bit
#define DISTANCE_REG_gp2y0e02b  0x5E
#define SHIFT_gp2y0e02b         0x35

class SensorsDriver: public ASensorsDriver
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref SensorsDriver(OPOS6UL).
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("SensorsDriver.OPO");
        return instance;
    }

    BeaconSensors beaconSensors_;
    Registers regs_;
    Settings settings_;

    bool beacon_connected_;

    MatchSettingsData cached_settings_;  ///< Dernier Settings lu par syncFull().
    bool settings_valid_ = false;        ///< true si dernier readSettings OK dans syncFull().

    // Dirty flags par champ : seuls les champs modifies sont ecrits en I2C.
    bool dirty_numOfBots_    = false;
    bool dirty_matchPoints_  = false;
    bool dirty_matchState_   = false;
    bool dirty_matchColor_   = false;
    bool dirty_strategy_     = false;
    bool dirty_advDiameter_  = false;
    bool dirty_actionReq_    = false;

//    Gp2y0e02b gp2_1_;
//    Gp2y0e02b gp2_2_;

    bool connected_gp2y0e02b_;

    utils::Mutex msync_;
    bot_positions vadv_;  //tableau des pos des adv

    ARobotPositionShared *robotpos_;  ///< Pour acceder au chrono partage (synchro beacon).
    uint32_t last_sync_ms_;           ///< Timestamp capture juste apres readFlag (= proche de la fin de cycle Teensy).


//    int getFrontDistMmFromObject(int diagonal_dist_mm);
//    int getBackDistMmFromObject(int diagonal_dist_mm);


public:

    /*!
     * \brief Constructor.
     */
    SensorsDriver(ARobotPositionShared * robotpos);

    /*!
     * \brief Destructor.
     */
    ~SensorsDriver();

    bool is_connected();

    void displayNumber(int number);
    void writeLedLuminosity(uint8_t lum);
    int getAnalogPinData();

    // --- Settings balise (override ASensorsDriver pour delegation BeaconSensors) ---
    bool readMatchSettings(MatchSettingsData& out) override;
    bool writeMatchColor(uint8_t c) override;
    bool writeStrategy(uint8_t s) override;
    bool writeAdvDiameter(uint8_t d) override;
    bool writeMatchState(uint8_t s) override;
    bool writeMatchPoints(uint8_t p) override;
    bool writeNumOfBots(int8_t n) override;
    bool writeActionReq(uint8_t v) override;



    int sync(); //synchronise les données avec la balise, return 0 if success, -1 if error. (match)
    int syncFull() override; // init: write pending settings + read settings + read flag+getData
    ASensorsDriver::bot_positions getvPositionsAdv(); //retourne les dernieres positions connues

    void addvPositionsAdv(float x, float y) ;
    void clearPositionsAdv() ;

    int frontLeft(); //retourne la dernière distance minimum gauche (en mm) apres le sync
    int frontCenter();
    int frontRight(); //retourne la dernière distance minimum droite (en mm) en mm apres le sync

    int backLeft();
    int backCenter();
    int backRight();

    int rightSide();
    int leftSide();

    uint32_t getBeaconSeq() override { return regs_.seq; }

    /*!
     * \brief Retourne le timestamp (ms) du dernier sync I2C reussi (juste apres readFlag).
     * \return Timestamp en ms depuis le chrono partage de RobotPositionShared.
     *         Plus precis que mesurer "avant sync()" car la Teensy a fini son cycle
     *         juste avant que readFlag ne renvoie 0x01 (nouvelle donnee disponible).
     */
    uint32_t getLastSyncMs() { return last_sync_ms_; }

};

#endif
