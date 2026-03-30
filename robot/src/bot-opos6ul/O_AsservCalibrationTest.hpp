/*!
 * \file
 * \brief Déclaration de la classe O_AsservCalibrationTest.
 */

#ifndef O_ASSERVCALIBRATIONTEST_HPP
#define O_ASSERVCALIBRATIONTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de calibration de l'asservissement.
 *
 * Permet de tester par étapes : codeurs (step 0-1), moteurs (step 2),
 * réglage P (step 3), réglage D distance (step 4), réglage D angle (step 5).
 */
class O_AsservCalibrationTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservCalibrationTest");
        return instance;
    }

public:

    O_AsservCalibrationTest() :
            FunctionalTest("Asserv_Calibration", "Calibration asserv par etapes")
    {
    }

    virtual ~O_AsservCalibrationTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
