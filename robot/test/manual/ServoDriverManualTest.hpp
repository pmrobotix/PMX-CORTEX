/*!
 * \file
 * \brief Définition de la classe ServoDriverManualTest.
 */

#ifndef OPOS6UL_SERVODRIVERMANUALTEST_HPP
#define OPOS6UL_SERVODRIVERMANUALTEST_HPP

#include "interface/AServoDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe \ref ServoDriverManualTest.
 */
class ServoDriverManualTest: public UnitTest
{
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref ServoDriverManualTest(OPO).
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("ServoDriverManualTest.OPO");
        return instance;
    }

    AServoDriver* servodriver_;

public:



    /*!
     * \brief Constructeur de la classe.
     */
    ServoDriverManualTest() :
            UnitTest("ServoDriverManualTest")
    {
        servodriver_ = AServoDriver::create();

    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~ServoDriverManualTest()
    {
    }

    virtual void suite();

    void firstTest();

};
}

#endif
