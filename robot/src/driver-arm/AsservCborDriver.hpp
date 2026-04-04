/*!
 * \file
 * \brief AsservCborDriver — Communication CBOR avec la Nucleo (SerialCbor).
 *
 * Remplace AsservDriver (protocole ASCII) par le protocole binaire CBOR
 * avec framing (sync word 0xDEADBEEF + CRC32 + taille + payload).
 * Utilisé par Princess/Picrate et EsialRobotik.
 */

#ifndef ASSERV_CBOR_DRIVER_HPP_
#define ASSERV_CBOR_DRIVER_HPP_

#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "thread/Thread.hpp"
#include "serialib.hpp"
#include "cbor/CborFrameDecoder.hpp"
#include <cstdint>
#include <mutex>

#define SERIAL_CBOR_PORT "/dev/ttymxc1"

class AsservCborDriver : public AAsservDriver, utils::Thread
{
private:
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("AsservCborDriver");
        return instance;
    }
    static inline const logs::Logger & loggerSvg()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("AsservCborDriver.SVG");
        return instance;
    }

    serialib serial_;
    CborFrameDecoder frameDecoder_;

    bool connected_;
    bool asservCardStarted_;
    bool threadStarted_;
    int errorCount_;
    int nextCmdId_;
    int lastReceivedCmdId_;
    int statusCountDown_;

    Mutex m_pos;
    Mutex m_statusCountDown;
    ROBOTPOSITION p_;
    ROBOTPOSITION pp_; // position précédente pour trace SVG

    // Envoi d'une commande CBOR framée (sync + CRC + taille + payload)
    void sendCmd(int cmdType);
    void sendCmd(int cmdType, float arg1);
    void sendCmd(int cmdType, float arg1, float arg2);
    void sendCmd(int cmdType, float arg1, float arg2, float arg3);

    // Envoi bas niveau de la trame complète
    void sendFrame(const uint8_t *cborPayload, size_t cborLen);

    void prepareCommand(int countdown);

    uint8_t txBuffer_[128];

protected:
    virtual void execute();

public:
    AsservCborDriver();
    ~AsservCborDriver();

    /*!
     * \brief Démarre le thread de réception série (position 10Hz).
     * À appeler après l'init du SVG (beginHeader).
     */
    void startReceiveThread();

    bool is_connected() override;
    void endWhatTodo();
    TRAJ_STATE waitEndOfTraj() override;

    // Bas niveau — stubs (utilisés par tests O_* uniquement)
    void setMotorLeftPower(int power, int time);
    void setMotorRightPower(int power, int time);
    void stopMotors();
    void getCountsExternal(int32_t* countR, int32_t* countL);
    void getDeltaCountsExternal(int32_t* deltaR, int32_t* deltaL);
    void resetEncoders();

    // Odométrie et status
    void odo_SetPosition(float x_mm, float y_mm, float angle_rad);
    ROBOTPOSITION odo_GetPosition();
    void emergencyStop();
    void resetEmergencyStop();

    // Commandes de mouvement (non-bloquantes)
    void motion_DoLine(float dist_mm);
    void motion_DoFace(float x_mm, float y_mm, bool back_reversed);
    void motion_DoRotate(float angle_radians);
    void motion_DoOrbitalTurn(float angle_radians, bool forward, bool turnRight);
    void motion_Goto(float x_mm, float y_mm);
    void motion_GotoReverse(float x_mm, float y_mm);
    void motion_GotoChain(float x_mm, float y_mm);
    void motion_GotoReverseChain(float x_mm, float y_mm);

    // Modes de mouvement
    void motion_FreeMotion(void);
    void motion_AssistedHandling(void);
    void motion_ActivateManager(bool enable);
    void motion_setLowSpeedForward(bool enable, int percent);
    void motion_setLowSpeedBackward(bool enable, int percent);
    void motion_setMaxSpeed(bool enable, int speed_dist_percent, int speed_angle_percent);
    void motion_ActivateReguDist(bool enable);
    void motion_ActivateReguAngle(bool enable);
};

#endif
