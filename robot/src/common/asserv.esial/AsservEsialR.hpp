#ifndef ASSERV_E_ASSERVESIALR_H_
#define ASSERV_E_ASSERVESIALR_H_

#include <sys/types.h>

#include "interface/AAsservDriver.hpp"
#include "utils/Chronometer.hpp"
#include "log/LoggerFactory.hpp"
#include "thread/Thread.hpp"

class CodeursInterface;
class CommandManagerA;
class ConsignController;
class MotorsController;
class Odometrie;

class Robot;
/*//TODO replace all uint / be careful with AsservInsa which already define them
 #define uint8 unsigned char
 #define uint16 unsigned short
 #define uint32 unsigned int
 #define int8 char
 #define int16 short
 #define int32 int
 //#define BOOL bool*/

class AsservEsialR: public AAsservDriver, public utils::Thread
{

protected:
     void execute();

private:
     bool loop_finished_;
     unsigned long long last_;

    /*!
     *\brief Chronometre lie.
     */
    utils::Chronometer chronoTimer_;

    /*!
     *\brief ref vers le robot lie.
     */
    Robot * robot_;

    AAsservDriver* asservdriver;

    bool run_;
    //bool loop_not_finished_ ;

    //nb of period since the beginning
    uint periodNb_;

    //loop delay
    uint loopDelayInMillisec_;

    CodeursInterface *codeurs_;
    Odometrie *odo_;
    MotorsController *motorC_;
    ConsignController *consignC_;
    CommandManagerA *commandM_;

    TRAJ_STATE pathStatus_;
    ROBOTPOSITION p_; //position du robot via cet asservissement

    /*!
     * \brief Retourne le \ref Logger associe a la classe \ref AsservEsialR.
     */
    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("AsservEsialR");
        return instance;
    }

    /*!
     * \brief Retourne le \ref Logger file associe a la classe \ref AsservEsialR.
     */
    static inline const logs::Logger & loggerFile() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("logFileAsservEsialR");
        return instance;
    }

public:
    AsservEsialR(Robot * robot);
    ~AsservEsialR() {
    }

    void setSamplingFrequency(uint frequency);
    void loadConfig();
    void startAsserv(int freqHz);
    void initAsserv();
    void stopAsserv();
    void resetAsserv();
    TRAJ_STATE waitEndOfTraj() override;
    int getAsservStatus();

    bool is_connected() override { return true; }
    void endWhatTodo();

    void odo_SetPosition(float x_mm, float y_mm, float angle_rad);
    ROBOTPOSITION odo_GetPosition();

    void emergencyStop();
    void resetEmergencyStop();

    void motion_Line(float dist_mm);
    void motion_FaceTo(float x_mm, float y_mm);
    void motion_FaceBackTo(float x_mm, float y_mm);
    void motion_RotateRad(float angle_radians);
    void motion_OrbitalTurnRad(float angle_radians, bool forward, bool turnRight);
    void motion_GoTo(float x_mm, float y_mm);
    void motion_GoBackTo(float x_mm, float y_mm);
    void motion_GoToChain(float x_mm, float y_mm);
    void motion_GoBackToChain(float x_mm, float y_mm);
    void motion_DoDirectLine(float dist_mm);

    void motion_FreeMotion(void);
    void motion_AssistedHandling(void);
    void motion_ActivateManager(bool enable);
    void motion_setLowSpeedForward(bool enable, int percent = 0);
    void motion_setLowSpeedBackward(bool enable, int percent = 0);
    void motion_setMaxSpeed(bool enable, int speed_dist_m_sec = 0, int speed_angle_rad_sec=0);

    void motion_ActivateReguDist(bool enable);
    void motion_ActivateReguAngle(bool enable);
    void motion_ResetReguDist();
    void motion_ResetReguAngle();

    void motion_ActivateQuadRamp(bool enable);

    // Stubs bas niveau (utilisés par tests O_* et Asserv.cpp)
    void setMotorLeftPower(int power, int time);
    void setMotorRightPower(int power, int time);
    void stopMotors();
    void getCountsExternal(int32_t* countR, int32_t* countL);
    void getDeltaCountsExternal(int32_t* deltaR, int32_t* deltaL);
    void resetEncoders();

};

#endif /* ASSERV_ESIALR_ASSERVESIALR_H_ */
