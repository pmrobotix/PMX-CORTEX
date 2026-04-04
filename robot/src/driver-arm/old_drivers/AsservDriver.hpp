/*!
 * \file
 * \brief Définition de la classe AsservDriver (OPOS6UL).
 */

#ifndef OPOS6UL_ASSERVDRIVER_HPP_
#define OPOS6UL_ASSERVDRIVER_HPP_



#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "thread/Thread.hpp"
//#include <include/CppLinuxSerial/SerialPort.hpp>
#include "serialib.hpp"
#include <unistd.h>
#include <stdio.h>
#include <vector>

//using namespace mn::CppLinuxSerial;

//#define	SERIAL_ADDRESS      "/dev/ttymxc1"
#define SERIAL_PORT "/dev/ttymxc1"

typedef enum {
            STATUS_IDLE     = 0,
            STATUS_RUNNING  = 1,
            STATUS_HALTED   = 2,
            STATUS_BLOCKED  = 3,
        } CommandStatus;

// convert float to byte array  source: http://mbed.org/forum/helloworld/topic/2053/
union float2bytes_t   // union consists of one variable represented in a number of different ways
{
    float f;
    unsigned char b[sizeof(float)];

    float2bytes_t() :
            b { }
    {
    } //initialisation
};

class AsservDriver: public AAsservDriver, utils::Thread
{

private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref AsservDriver(OPOS6UL).
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("AsservDriver.OPO");
        return instance;
    }
    static inline const logs::Logger & loggerSvg()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("AsservDriver.OPO.SVG");
        return instance;
    }

    //SerialPort serialPort_;
    serialib serial_;

    int read_error_;
    bool connected_;
    int errorCount_;

    bool asservCardStarted_;

    int statusCountDown_;

    //TRAJ_STATE pathStatus_;

    Mutex m_pos; //mutex pour la mise à jour de la position
    Mutex m_statusCountDown;

    int nucleo_flushSerial();
    int nucleo_writeSerial(char c);
    int nucleo_writeSerialSTR(std::string str);


    void parseAsservPosition(std::string str);

    void tokenize(std::string const &str, const char delim, std::vector<std::string> &out);

    ROBOTPOSITION pp_; //position precedente
protected:

    virtual void execute();
    ROBOTPOSITION p_;
public:

    bool is_connected() override;
    void endWhatTodo();

    // Stubs bas niveau (utilisés par tests O_* et Asserv.cpp)
    void setMotorLeftPower(int power, int time);
    void setMotorRightPower(int power, int time);
    void stopMotors();
    void getCountsExternal(int32_t* countR, int32_t* countL){}
    void getDeltaCountsExternal(int32_t* deltaR, int32_t* deltaL){}
    void resetEncoders();

    //fonctions asservissements externe par defaut
    void odo_SetPosition(float x_mm, float y_mm, float angle_rad);
    ROBOTPOSITION odo_GetPosition();
    void emergencyStop();

    void resetEmergencyStop();
    TRAJ_STATE waitEndOfTraj() override;

    void motion_Line(float dist_mm);
    void motion_FaceTo(float x_mm, float y_mm);
    void motion_FaceBackTo(float x_mm, float y_mm);
    void motion_RotateRad(float angle_radians);
    void motion_OrbitalTurnRad(float angle_radians, bool forward, bool turnRight);
    void motion_GoTo(float x_mm, float y_mm);
    void motion_GoBackTo(float x_mm, float y_mm);
    void motion_GoToChain(float x_mm, float y_mm);
    void motion_GoBackToChain(float x_mm, float y_mm);

    void motion_FreeMotion(void);
    void motion_AssistedHandling(void);
    void motion_ActivateManager(bool enable);
    void motion_setLowSpeedForward(bool enable, int percent); //TODO remove enable
    void motion_setLowSpeedBackward(bool enable, int percent); //TODO remove enable
    void motion_setMaxSpeed(bool enable, int speed_dist_percent = 0, int speed_angle_percent=0);

    //Functions deprecated
    void motion_ActivateReguDist(bool enable);
    void motion_ActivateReguAngle(bool enable);
  

    /*!
     * \brief Constructor.
     */
    AsservDriver();

    /*!
     * \brief Destructor.
     */
    ~AsservDriver();

};

#endif
