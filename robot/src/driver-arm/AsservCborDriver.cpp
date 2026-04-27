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
#include "utils/json.hpp"

using namespace std;

// Factory AsservDriverSimu + AsservCborDriver (protocole CBOR)
AAsservDriver* AAsservDriver::create(string botid, ARobotPositionShared *robotpos)
{
    if (HardwareConfig::instance().isEnabled("AsservCborDriver")) {
        static AsservCborDriver *instance = new AsservCborDriver(robotpos);
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
    CMD_SET_SPEED_PERCENT    = 18,
    CMD_TURN                 = 20,
    CMD_STRAIGHT             = 21,
    CMD_FACE                 = 22,
    CMD_GOTO_FRONT           = 23,
    CMD_GOTO_BACK            = 24,
    CMD_GOTO_NOSTOP          = 25,
    CMD_FACE_BACK            = 27,
    CMD_GOTO_BACK_NOSTOP     = 28,
    CMD_ORBITAL_TURN         = 30,
    CMD_SET_POSITION         = 31,   // Deplace de 26 a 31 (master 3a2c3de) :
                                     // four-param (3 floats X/Y/theta + ID)
                                     // au lieu de three-param (2 floats).
};

static constexpr uint32_t SYNC_WORD = 0xDEADBEEF;

// ============================================================================
// Construction / Destruction
// ============================================================================

AsservCborDriver::AsservCborDriver(ARobotPositionShared *sharedPosition)
    : connected_(true), asservCardStarted_(true), threadStarted_(false),
      stopRequested_(false), positionInitialized_(false),
      // nextCmdId_ demarre haut (10000) pour garantir que nos motions ont un
      // cmd_id strictement superieur a tout residuel m_current_index Nucleo
      // (typiquement < 100 sur un run normal). Evite le faux ACK
      // "received_cmd=6 >= target=1 AND status=IDLE" -> traj jamais demarree
      // mais consideree comme terminee. Pas de sync dynamique necessaire.
      errorCount_(0), nextCmdId_(10000), lastReceivedCmdId_(0),
      positionFrameCounter_(0),
      p_({0.0, 0.0, 0.0, 0, 0, 0}), pp_({0.0, 0.0, 0.0, 0, 0, 0}),
      sharedPosition_(sharedPosition)
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

void AsservCborDriver::stopReceiveThread()
{
    if (!threadStarted_) return;
    stopRequested_ = true;
    // Le thread sortira de readBytes (timeout 50ms) puis verra le flag
    this->waitForEnd();
    threadStarted_ = false;
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

bool AsservCborDriver::tryReconnect()
{
    if (asservCardStarted_) return true;

    // Reouvre le port serie. Si openDevice retourne autre chose que 1, on log
    // et on revient avec false : l'appelant retentera plus tard.
    char errorOpening = serial_.openDevice(SERIAL_CBOR_PORT, 115200);
    if ((int)errorOpening != 1) {
        logger().warn() << "tryReconnect: openDevice=" << (int)errorOpening
                << " (Nucleo non branchee ou non alimentee ?)" << logs::end;
        return false;
    }

    serial_.DTR(true);
    serial_.RTS(false);
    serial_.flushReceiver();

    // On attend une trame CBOR de la Nucleo dans les 500ms.
    uint8_t buf[64];
    int bytesRead = serial_.readBytes(buf, sizeof(buf), 500);
    if (bytesRead <= 0) {
        logger().warn() << "tryReconnect: pas de reponse sur " << SERIAL_CBOR_PORT
                << " (Nucleo en attente de reset ?)" << logs::end;
        serial_.closeDevice();
        return false;
    }

    // On a vu au moins 1 frame -> serial OK et Nucleo emet. MAIS les bytes lus
    // peuvent etre en plein milieu d'une trame (sync word pas encore aligne)
    // ou des residus boot Nucleo. On JETTE ces bytes au lieu de les pousser
    // dans frameDecoder_ pour eviter qu'il reste sur un etat partiel pollue
    // au moment ou les vraies commandes vont commencer (set_position, LINE).
    // Pareil pour le buffer Rx serie.
    serial_.flushReceiver();
    frameDecoder_.reset();

    asservCardStarted_ = true;
    logger().info() << "tryReconnect: OK sur " << SERIAL_CBOR_PORT
            << " (jete " << bytesRead << " bytes initiaux + reset decoder)"
            << logs::end;

    // Demarre le thread de reception (idempotent grace au flag threadStarted_).
    startReceiveThread();
    return true;
}

void AsservCborDriver::endWhatTodo()
{
    emergencyStop();
    // Arret propre du thread de reception : evite que des positions soient
    // ecrites dans le SVG apres la balise </svg> de fermeture.
    stopReceiveThread();
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

// Helper pour afficher le nom de la cmd CBOR dans les logs (cf CborStreamStateMachine.h)
static const char* cborCmdName(int cmdType)
{
    switch (cmdType) {
        case 10: return "emergency_stop";
        case 11: return "emergency_stop_reset";
        case 15: return "normal_speed_acc";
        case 16: return "slow_speed_acc";
        case 17: return "max_motor_speed";
        case 18: return "set_speed_percent";
        case 20: return "turn";
        case 21: return "straight";
        case 22: return "face";
        case 23: return "goto_front";
        case 24: return "goto_back";
        case 25: return "goto_nostop";
        case 27: return "face_back";
        case 28: return "goto_back_nostop";
        case 30: return "orbital_turn";
        case 31: return "set_position";
        default: return "?";
    }
}

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
    logger().info() << "CBOR send cmd=" << cmdType << " (" << cborCmdName(cmdType) << ")" << logs::end;
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
    logger().info() << "CBOR send cmd=" << cmdType << " (" << cborCmdName(cmdType)
            << ") arg1=" << arg1 << " id=" << cmdId << logs::end;
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
    logger().info() << "CBOR send cmd=" << cmdType << " (" << cborCmdName(cmdType)
            << ") X=" << arg1 << " Y=" << arg2 << " id=" << cmdId << logs::end;
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
    logger().info() << "CBOR send cmd=" << cmdType << " (" << cborCmdName(cmdType)
            << ") X=" << arg1 << " Y=" << arg2 << " T=" << arg3 << " id=" << cmdId << logs::end;
}

// ============================================================================
// Thread de réception (position updates de la Nucleo)
// ============================================================================

void AsservCborDriver::execute()
{
    logger().info() << "AsservCborDriver::execute() — receive thread started" << logs::end;
    uint8_t rxBuf[64];

    while (!asservCardStarted_ && !stopRequested_)
        utils::sleep_for_micros(10000);

    while (!stopRequested_)
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
                ROBOTPOSITION p_copy = p_;
                m_pos.unlock();

                // Mise a jour du dernier cmd_id ACQ par la Nucleo. Atomique :
                // lu sans lock par Asserv::waitEnd (handshake cmd_id, voir la
                // doc dans AAsservDriver.hpp). Le HACK statusCountDown_ qui
                // forcait status=1 sur les 1ers IDLE post-send a ete retire :
                // avec le cmd_id check, un IDLE stale (lastReceived < lastSent)
                // ne sera plus interprete a tort comme "commande terminee".
                lastReceivedCmdId_.store(pos.cmd_id);

                // Compteur monotone de frames recues. Utilise par
                // Asserv::setPositionReal (handshake set_position) pour
                // detecter "une frame fraiche est arrivee depuis le send"
                // sans se fier a la valeur de pose elle-meme.
                positionFrameCounter_.fetch_add(1);

                // IMPORTANT : ignorer les positions recues AVANT le premier odo_SetPosition().
                // Tant que positionInitialized_ == false, la Nucleo envoie encore les positions
                // residuelles de la session precedente (non resetees). Les logger dans le SVG
                // ou dans sharedPosition creerait de fausses donnees au debut du match.
                if (!positionInitialized_) {
                    continue;
                }

                // Push history pour synchronisation beacon (SensorsThread/getPositionAt)
                // Le thread CBOR est le seul a recevoir les positions de la Nucleo,
                // donc seul lui doit alimenter le buffer historique.
                if (sharedPosition_ != nullptr) {
                    sharedPosition_->setRobotPosition(p_copy);
                }

                // Telemetrie UDP : position du robot
                // Trame: {"OPOS6UL":{"t":...,"dt":...,"Pos":{"x":150.0,"y":300.0,"a":90.0,"s":1,"q":2}}}
                {
                    static const logs::Logger & logPos = logs::LoggerFactory::logger("Pos");
                    nlohmann::json j;
                    j["x"] = p_copy.x;
                    j["y"] = p_copy.y;
                    j["a"] = p_copy.theta * (180.0f / M_PI); // rad -> deg
                    j["s"] = p_copy.asservStatus;
                    j["q"] = p_copy.queueSize;
                    logPos.telemetry(j.dump());
                }

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
// waitForCmdAck — Phase A du handshake (poll lastReceivedCmdId_)
// ============================================================================
//
// Apres l'envoi d'une commande motion, on attend ici que la Nucleo l'ait
// effectivement consommee (m_current_index = notre cmd_id, echo dans la
// frame positionOutput).
//
// Si timeout : la commande a probablement ete perdue par overflow Rx Nucleo
// (le buffer hardware deborde quand AsservMain::onTimer ou commandInput
// tient le CPU). L'appelant (Asserv::sendCborMotionWithRetry) peut retry.
//
// Timeout typique 200ms = 2x periode positionOutput (100ms) + marge.
// Polling 5ms : reactivite + faible charge CPU.
//
bool AsservCborDriver::waitForCmdAck(int targetCmdId, int timeout_ms)
{
    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        if (lastReceivedCmdId_.load() >= targetCmdId) return true;
        utils::sleep_for_micros(5000); // 5ms
        elapsed_ms += 5;
    }
    return false;
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
// Commandes de mouvement (non-bloquantes : envoient la commande et retournent)
// Appeler waitEndOfTraj() séparément pour attendre la fin.
// ============================================================================

void AsservCborDriver::prepareCommand()
{
    // Defensif : on force localement asservStatus=RUNNING avant l'envoi,
    // pour que la 1ere frame CBOR (qui peut encore reporter IDLE de la
    // commande precedente) ne fasse pas voir un faux "termine" entre l'envoi
    // et le reset par la Nucleo. Le critere d'autorite reste le cmd_id ack
    // (Asserv::waitEnd attend lastReceivedCmdId() >= lastSentCmdId()).
    m_pos.lock(); p_.asservStatus = 1; m_pos.unlock();
}

void AsservCborDriver::motion_Line(float dist_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_STRAIGHT, dist_mm);
}

void AsservCborDriver::motion_RotateRad(float angle_radians)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_TURN, (float)(angle_radians * 180.0 / M_PI));
}

void AsservCborDriver::motion_FaceTo(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_FACE, x_mm, y_mm);
}

void AsservCborDriver::motion_FaceBackTo(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_FACE_BACK, x_mm, y_mm);
}

void AsservCborDriver::motion_GoTo(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_GOTO_FRONT, x_mm, y_mm);
}

void AsservCborDriver::motion_GoBackTo(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_GOTO_BACK, x_mm, y_mm);
}

void AsservCborDriver::motion_GoToChain(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_GOTO_NOSTOP, x_mm, y_mm);
}

void AsservCborDriver::motion_GoBackToChain(float x_mm, float y_mm)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    sendCmd(CMD_GOTO_BACK_NOSTOP, x_mm, y_mm);
}

void AsservCborDriver::motion_OrbitalTurnRad(float angle_radians, bool forward, bool turnRight)
{
    if (!asservCardStarted_) return;
    prepareCommand();
    float angleDeg = (float)(angle_radians * 180.0 / M_PI);
    sendCmd(CMD_ORBITAL_TURN, angleDeg, forward ? 1.0f : 0.0f, turnRight ? 1.0f : 0.0f);
}

// ============================================================================
// Odométrie et status
// ============================================================================

void AsservCborDriver::odo_SetPosition(float x_mm, float y_mm, float angle_rad)
{
    sendCmd(CMD_SET_POSITION, x_mm, y_mm, angle_rad);
    // Apres ce premier set position, les positions recues de la Nucleo seront
    // valides (pas des residus de la session precedente) et peuvent etre loggees.
    positionInitialized_ = true;
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
// resetNucleoState — flush queue motion residuelle Nucleo au boot brain
// ============================================================================
//
// Entre 2 boots du brain, la Nucleo conserve sa queue motion. Sans ce flush,
// les motions du run precedent s'executent au prochain start -> robot bouge
// tout seul de 100mm en 100mm.
//
// Solution : 2 SENDS CBOR existants. Pas de lecture (pas besoin du receive
// thread, qui n'est pas encore demarre a ce stade). Le faux ACK lie au
// m_current_index residuel est neutralise par nextCmdId_(10000) au ctor.
//
bool AsservCborDriver::resetNucleoState()
{
    if (!asservCardStarted_) {
        logger().warn() << "resetNucleoState: Nucleo non connectee, skip" << logs::end;
        return false;
    }

    // 1. Demarre le receive thread tot, AVANT toute lecture de positionOutput
    //    (sans lui, les frames CBOR arrivent sur la serie mais personne ne les
    //    lit -> impossible de connaitre m_current_index Nucleo et d'eviter
    //    les faux ACK sur les motions a venir).
    startReceiveThread();

    // 2. Flush la queue motion residuelle Nucleo (cmds non executees du run
    //    precedent du brain). m_current_index Nucleo reste a sa valeur
    //    courante (incremental, on n'y touche pas).
    sendCmd(CMD_EMERGENCY_STOP);
    utils::sleep_for_micros(50000); // 50ms : flush
    sendCmd(CMD_EMERGENCY_STOP_RESET);

    // 3. Attendre la 1ere frame positionOutput pour lire m_current_index
    //    Nucleo. Permet de caler nextCmdId_ au-dessus -> aucune motion brain
    //    ne sera faussement consideree comme deja terminee par
    //    waitEndOfTrajWithDetection (received_cmd_id >= target_cmd_id).
    constexpr int TIMEOUT_MS = 1500;
    constexpr int POLL_PERIOD_MS = 20;
    constexpr int SAFETY_OFFSET = 100;   // marge si cmds en vol au moment du sync
    const int counterBefore = positionFrameCounter_.load();
    int elapsed_ms = 0;
    while (elapsed_ms < TIMEOUT_MS) {
        utils::sleep_for_micros(POLL_PERIOD_MS * 1000);
        elapsed_ms += POLL_PERIOD_MS;
        if (positionFrameCounter_.load() > counterBefore) {
            const int nucleoCurrentIndex = lastReceivedCmdId_.load();
            const int newNextCmdId = nucleoCurrentIndex + SAFETY_OFFSET;
            nextCmdId_.store(newNextCmdId);
            logger().info() << "resetNucleoState: OK Nucleo m_current_index="
                    << nucleoCurrentIndex << " -> nextCmdId_=" << newNextCmdId
                    << " (apres " << elapsed_ms << "ms)" << logs::end;
            return true;
        }
    }

    logger().error() << "resetNucleoState: TIMEOUT " << TIMEOUT_MS
            << "ms sans frame positionOutput - Nucleo absente ou serielle KO ?"
            << logs::end;
    return false;
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

void AsservCborDriver::motion_setAccDecPercent(int percent)
{
    sendCmd(CMD_SET_SPEED_PERCENT, (float)percent);
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
