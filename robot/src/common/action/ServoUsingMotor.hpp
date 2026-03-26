/*! \file
 * \brief Définition de la classe ServoUsingMotor.
 */

#ifndef SERVOUSINGMOTOR_HPP_
#define SERVOUSINGMOTOR_HPP_

#include <string>

#include "log/LoggerFactory.hpp"
#include "AActionsElement.hpp"

class AServoUsingMotorDriver;

/*!
 * \brief Gère un servomoteur piloté via un driver moteur.
 */
class ServoUsingMotor: public AActionsElement {
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref ServoUsingMotor.
     */
    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("ServoUsingMotor");
        return instance;
    }

    AServoUsingMotorDriver* servoMotordriver_;

    //int ticks_place;

    /*!
     * \brief ID du robot.
     */
    std::string botId_;

public:

    /*!
     * \brief Constructor.
     */
    ServoUsingMotor(std::string botId, Actions & actions);

    /*!
     * \brief Destructor.
     */
    ~ServoUsingMotor();

    /*!
     * \brief Vérifie si le servomoteur est connecté.
     */
    bool is_connected();

    /*!
     * \brief Fait tourner le servomoteur à la vitesse indiquée.
     * \param speed Vitesse de rotation.
     */
    void turn(int speed);

    /*!
     * \brief Arrête le servomoteur.
     */
    void stop();
//    void moveRight(int);
//    void moveLeft(int);
//    void ejectRight();
//    void ejectLeft();

};

#endif
