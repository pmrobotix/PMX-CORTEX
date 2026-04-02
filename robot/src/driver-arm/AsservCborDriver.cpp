// AsservCborDriver — Communication CBOR avec la Nucleo
// Protocole binaire : sync 0xDEADBEEF + CRC32 + taille + payload CBOR
// Ref: asserv_chibios/src/Communication/SerialCbor.cpp (côté Nucleo)
//      github.com/EsialRobotik/Ia-Asserv_Com (côté Brain Python)

#include "AsservCborDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/AsservDriver.hpp"
#include "log/Logger.hpp"

#include <cmath>
#include <cstring>
#include <qcbor/qcbor_encode.h>
#include "cppCrc.h"

using namespace std;

// Factory AsservDriverSimu + AsservCborDriver (protocole CBOR)
AAsservDriver* AAsservDriver::create(string botid, ARobotPositionShared *robotpos)
{
    if (HardwareConfig::instance().isEnabled("AsservCborDriver")) {
        static AsservCborDriver *instance = new AsservCborDriver();
        return instance;
    }
    // Fallback simulateur
    static AsservDriverSimu *instance = new AsservDriverSimu(botid, robotpos);
    return instance;
}

// Command types — doit matcher CborStreamStateMachine::cmd_type_t côté Nucleo
enum CborCmdType {
    CMD_EMERGENCY_STOP       = 10,
    CMD_EMERGENCY_STOP_RESET = 11,
    CMD_NORMAL_SPEED_ACC     = 15,
    CMD_SLOW_SPEED_ACC       = 16,
    CMD_MAX_MOTOR_SPEED      = 17,
    CMD_TURN                 = 20,
    CMD_STRAIGHT             = 21,
    CMD_FACE                 = 22,
    CMD_GOTO_FRONT           = 23,
    CMD_GOTO_BACK            = 24,
    CMD_GOTO_NOSTOP          = 25,
    CMD_SET_POSITION         = 26,
    CMD_ORBITAL_TURN         = 30,
};

static constexpr uint32_t SYNC_WORD = 0xDEADBEEF;

// ============================================================================
// Construction / Destruction
// ============================================================================

AsservCborDriver::AsservCborDriver()
    : connected_(true), asservCardStarted_(true), threadStarted_(false), errorCount_(0),
      nextCmdId_(1), lastReceivedCmdId_(0), statusCountDown_(0),
      p_({0.0, 0.0, 0.0, 0, 0, 0}), pp_({0.0, 0.0, 0.0, 0, 0, 0})
{
    char errorOpening = serial_.openDevice(SERIAL_CBOR_PORT, 115200);
    if ((int)errorOpening != 1)
    {
        logger().error() << "AsservCborDriver: NOT connected on " << SERIAL_CBOR_PORT
                         << " error=" << (int)errorOpening << logs::end;
        asservCardStarted_ = false;
    }
    else
    {
        serial_.DTR(true);
        serial_.RTS(false);
        serial_.flushReceiver();

        // Pas de handshake "###" en mode CBOR — le framing avec sync word suffit.
        // On attend une première trame position de la Nucleo pour confirmer la connexion.
        uint8_t buf[64];
        int bytesRead = serial_.readBytes(buf, sizeof(buf), 500);
        if (bytesRead <= 0)
        {
            logger().error() << "AsservCborDriver: NOT responding on " << SERIAL_CBOR_PORT << logs::end;
            asservCardStarted_ = false;
            serial_.closeDevice();
        }
        else
        {
            // Pousser les premiers octets dans le décodeur
            for (int i = 0; i < bytesRead; i++)
                frameDecoder_.pushByte(buf[i]);
            logger().info() << "AsservCborDriver: OK on " << SERIAL_CBOR_PORT << logs::end;
            // Le thread de réception est démarré explicitement via startReceiveThread()
            // après l'init du SVG (beginHeader), pour éviter d'écrire dans le SVG avant le header.
        }
    }
}

AsservCborDriver::~AsservCborDriver()
{
    serial_.closeDevice();
}

void AsservCborDriver::startReceiveThread()
{
    if (asservCardStarted_ && !threadStarted_) {
        threadStarted_ = true;
        this->start("AsservCborDriver", 80);
    }
}

bool AsservCborDriver::is_connected()
{
    return connected_ && asservCardStarted_;
}

void AsservCborDriver::endWhatTodo()
{
    emergencyStop();
}

// ============================================================================
// Envoi de commandes CBOR
// ============================================================================

void AsservCborDriver::sendFrame(const uint8_t *cborPayload, size_t cborLen)
{
    uint32_t crc = CRC32::CRC32::calc(cborPayload, cborLen);
    uint32_t size = (uint32_t)cborLen;

    // Construire la trame : sync + crc + size + payload
    uint8_t frame[12 + 128];
    memcpy(frame, &SYNC_WORD, 4);
    memcpy(frame + 4, &crc, 4);
    memcpy(frame + 8, &size, 4);
    memcpy(frame + 12, cborPayload, cborLen);

    serial_.writeBytes(frame, 12 + cborLen);
}

// Le Nucleo lit les valeurs CBOR dans l'ordre : cmd_type, [arg1, [arg2, [arg3]]], cmd_id
// Les clés de la map sont ignorées par le décodeur Nucleo (lecture positionnelle).

void AsservCborDriver::sendCmd(int cmdType)
{
    uint8_t buf[64];
    QCBOREncodeContext ctx;
    UsefulBuf output = {buf, sizeof(buf)};
    QCBOREncode_Init(&ctx, output);
    QCBOREncode_OpenMap(&ctx);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "cmd", cmdType);
    QCBOREncode_CloseMap(&ctx);
    UsefulBufC encoded;
    if (QCBOREncode_Finish(&ctx, &encoded) == QCBOR_SUCCESS)
        sendFrame((const uint8_t *)encoded.ptr, encoded.len);
}

void AsservCborDriver::sendCmd(int cmdType, float arg1)
{
    int cmdId = nextCmdId_++;
    uint8_t buf[64];
    QCBOREncodeContext ctx;
    UsefulBuf output = {buf, sizeof(buf)};
    QCBOREncode_Init(&ctx, output);
    QCBOREncode_OpenMap(&ctx);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "cmd", cmdType);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "D", (double)arg1);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "ID", cmdId);
    QCBOREncode_CloseMap(&ctx);
    UsefulBufC encoded;
    if (QCBOREncode_Finish(&ctx, &encoded) == QCBOR_SUCCESS)
        sendFrame((const uint8_t *)encoded.ptr, encoded.len);
}

void AsservCborDriver::sendCmd(int cmdType, float arg1, float arg2)
{
    int cmdId = nextCmdId_++;
    uint8_t buf[64];
    QCBOREncodeContext ctx;
    UsefulBuf output = {buf, sizeof(buf)};
    QCBOREncode_Init(&ctx, output);
    QCBOREncode_OpenMap(&ctx);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "cmd", cmdType);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "X", (double)arg1);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "Y", (double)arg2);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "ID", cmdId);
    QCBOREncode_CloseMap(&ctx);
    UsefulBufC encoded;
    if (QCBOREncode_Finish(&ctx, &encoded) == QCBOR_SUCCESS)
        sendFrame((const uint8_t *)encoded.ptr, encoded.len);
}

void AsservCborDriver::sendCmd(int cmdType, float arg1, float arg2, float arg3)
{
    int cmdId = nextCmdId_++;
    uint8_t buf[64];
    QCBOREncodeContext ctx;
    UsefulBuf output = {buf, sizeof(buf)};
    QCBOREncode_Init(&ctx, output);
    QCBOREncode_OpenMap(&ctx);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "cmd", cmdType);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "X", (double)arg1);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "Y", (double)arg2);
    QCBOREncode_AddDoubleToMapSZ(&ctx, "T", (double)arg3);
    QCBOREncode_AddInt64ToMapSZ(&ctx, "ID", cmdId);
    QCBOREncode_CloseMap(&ctx);
    UsefulBufC encoded;
    if (QCBOREncode_Finish(&ctx, &encoded) == QCBOR_SUCCESS)
        sendFrame((const uint8_t *)encoded.ptr, encoded.len);
}

// ============================================================================
// Thread de réception (position updates de la Nucleo)
// ============================================================================

void AsservCborDriver::execute()
{
    logger().info() << "AsservCborDriver::execute() — receive thread started" << logs::end;
    uint8_t rxBuf[64];

    while (!asservCardStarted_)
        utils::sleep_for_micros(10000);

    while (true)
    {
        // Lire 1 octet avec timeout, puis vider le buffer disponible
        int bytesRead = serial_.readBytes(rxBuf, 1, 50);
        if (bytesRead == 1) {
            int avail = serial_.available();
            if (avail > 0) {
                if (avail > (int)sizeof(rxBuf) - 1) avail = sizeof(rxBuf) - 1;
                int more = serial_.readBytes(rxBuf + 1, avail, 10);
                if (more > 0) bytesRead += more;
            }
        }
        if (bytesRead > 0)
        {
            // printf("[CBOR] received %d bytes: ", bytesRead);
            // for (int i = 0; i < (bytesRead < 16 ? bytesRead : 16); i++)
            //     printf("%02X ", rxBuf[i]);
            // printf("\n");
            // fflush(stdout);
            for (int i = 0; i < bytesRead; i++)
                frameDecoder_.pushByte(rxBuf[i]);

            CborFrameDecoder::PositionData pos;
            while (frameDecoder_.getPosition(pos))
            {
                // printf("[CBOR] RX x=%d y=%d s=%d cmd=%d q=%d\n",
                //     pos.x, pos.y, pos.status, pos.cmd_id, pos.pending_count);
                // fflush(stdout);

                m_pos.lock();
                p_.x = (float)pos.x;
                p_.y = (float)pos.y;
                p_.theta = pos.theta;
                p_.asservStatus = pos.status;
                p_.queueSize = pos.pending_count;
                lastReceivedCmdId_ = pos.cmd_id;

                // Anti-race : ignorer les premiers IDLE après envoi d'une commande
                m_statusCountDown.lock();
                if (pos.status == 0 && statusCountDown_ > 0)
                {
                    statusCountDown_--;
                    p_.asservStatus = 1; // Forcer RUNNING
                }
                m_statusCountDown.unlock();

                m_pos.unlock();

                // Trace SVG : un point bleu + direction si la position a changé
                if (!(p_.x == pp_.x && p_.y == pp_.y))
                {
                    loggerSvg().info() << "<circle cx=\"" << p_.x << "\" cy=\"" << -p_.y
                            << "\" r=\"1\" fill=\"blue\" />" << "<line x1=\"" << p_.x << "\" y1=\"" << -p_.y
                            << "\" x2=\"" << p_.x + cos(p_.theta) * 25 << "\" y2=\"" << -p_.y - sin(p_.theta) * 25
                            << "\" stroke-width=\"0.1\" stroke=\"grey\"  />" << logs::end;
                    pp_ = p_;
                }
            }
        }
        else if (bytesRead < 0)
        {
            logger().error() << "AsservCborDriver: serial read error" << logs::end;
            asservCardStarted_ = false;
            break;
        }
    }
}

// ============================================================================
// waitEndOfTraj — poll status + cmd_id
// ============================================================================

TRAJ_STATE AsservCborDriver::waitEndOfTraj()
{
    utils::sleep_for_micros(100000); // 100ms initial

    int timeout = 0;
    while (true)
    {
        utils::sleep_for_micros(50000); // 50ms

        m_pos.lock();
        int status = p_.asservStatus;
        int queueSize = p_.queueSize;
        m_pos.unlock();

        // if (timeout % 20 == 0) {
        //     printf("[CBOR] waitEndOfTraj: status=%d queue=%d timeout=%d\n", status, queueSize, timeout);
        //     fflush(stdout);
        // }

        if (status != 1) // Plus RUNNING
        {
            if (status == 0 && queueSize == 0)
                return TRAJ_FINISHED;
            else if (status == 3)
                return TRAJ_COLLISION;
            else if (status == 2)
                return TRAJ_INTERRUPTED;
        }

        timeout++;
        if (timeout > 600) // 30 secondes
        {
            logger().error() << "AsservCborDriver: waitEndOfTraj TIMEOUT" << logs::end;
            return TRAJ_ERROR;
        }
    }
}

// ============================================================================
// Commandes de mouvement
// ============================================================================

TRAJ_STATE AsservCborDriver::motion_DoLine(float dist_mm)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    sendCmd(CMD_STRAIGHT, dist_mm);
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_DoRotate(float angle_radians)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    sendCmd(CMD_TURN, (float)(angle_radians * 180.0 / M_PI));
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_DoFace(float x_mm, float y_mm, bool back_reversed)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    // TODO: back_reversed non supporté par SerialCbor (face=22 ne gère pas le reverse)
    sendCmd(CMD_FACE, x_mm, y_mm);
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_Goto(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    sendCmd(CMD_GOTO_FRONT, x_mm, y_mm);
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_GotoReverse(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    sendCmd(CMD_GOTO_BACK, x_mm, y_mm);
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_GotoChain(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 5; m_statusCountDown.unlock();
    sendCmd(CMD_GOTO_NOSTOP, x_mm, y_mm);
    return waitEndOfTraj();
}

TRAJ_STATE AsservCborDriver::motion_GotoReverseChain(float x_mm, float y_mm)
{
    // Pas de commande CBOR pour goto reverse chain — non supporté par SerialCbor
    logger().error() << "motion_GotoReverseChain: not supported in CBOR protocol" << logs::end;
    return TRAJ_ERROR;
}

TRAJ_STATE AsservCborDriver::motion_DoOrbitalTurn(float angle_radians, bool forward, bool turnRight)
{
    if (!asservCardStarted_) return TRAJ_ERROR;
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
    m_statusCountDown.lock(); statusCountDown_ = 2; m_statusCountDown.unlock();
    float angleDeg = (float)(angle_radians * 180.0 / M_PI);
    sendCmd(CMD_ORBITAL_TURN, angleDeg, forward ? 1.0f : 0.0f, turnRight ? 1.0f : 0.0f);
    return waitEndOfTraj();
}

// ============================================================================
// Odométrie et status
// ============================================================================

void AsservCborDriver::odo_SetPosition(float x_mm, float y_mm, float angle_rad)
{
    sendCmd(CMD_SET_POSITION, x_mm, y_mm, angle_rad);
}

ROBOTPOSITION AsservCborDriver::odo_GetPosition()
{
    m_pos.lock();
    ROBOTPOSITION pos = p_;
    m_pos.unlock();
    return pos;
}

void AsservCborDriver::emergencyStop()
{
    sendCmd(CMD_EMERGENCY_STOP);
}

void AsservCborDriver::resetEmergencyStop()
{
    sendCmd(CMD_EMERGENCY_STOP_RESET);
}

// ============================================================================
// Modes de mouvement
// ============================================================================

void AsservCborDriver::motion_FreeMotion(void)
{
    sendCmd(CMD_EMERGENCY_STOP);
}

void AsservCborDriver::motion_AssistedHandling(void)
{
    sendCmd(CMD_EMERGENCY_STOP_RESET);
}

void AsservCborDriver::motion_ActivateManager(bool enable)
{
    if (enable)
    {
        startReceiveThread();
        sendCmd(CMD_EMERGENCY_STOP_RESET);
    }
    else
    {
        sendCmd(CMD_EMERGENCY_STOP);
    }
}

void AsservCborDriver::motion_setLowSpeedForward(bool enable, int percent)
{
    sendCmd(CMD_MAX_MOTOR_SPEED, (float)percent);
}

void AsservCborDriver::motion_setLowSpeedBackward(bool enable, int percent)
{
    sendCmd(CMD_MAX_MOTOR_SPEED, (float)percent);
}

void AsservCborDriver::motion_setMaxSpeed(bool enable, int speed_dist_percent, int speed_angle_percent)
{
    if (enable)
        sendCmd(CMD_MAX_MOTOR_SPEED, (float)speed_dist_percent);
    else
        sendCmd(CMD_NORMAL_SPEED_ACC);
}

void AsservCborDriver::motion_ActivateReguDist(bool enable)
{
    // Non supporté par le protocole CBOR — stub
}

void AsservCborDriver::motion_ActivateReguAngle(bool enable)
{
    // Non supporté par le protocole CBOR — stub
}

// ============================================================================
// Stubs bas niveau (utilisés par tests O_* uniquement)
// ============================================================================

void AsservCborDriver::setMotorLeftPower(int, int) {}
void AsservCborDriver::setMotorRightPower(int, int) {}
void AsservCborDriver::stopMotors() { motion_FreeMotion(); } // deprecated — utiliser motion_FreeMotion() directement
void AsservCborDriver::getCountsExternal(int32_t*, int32_t*) {}
void AsservCborDriver::getDeltaCountsExternal(int32_t*, int32_t*) {}
void AsservCborDriver::resetEncoders() {}
