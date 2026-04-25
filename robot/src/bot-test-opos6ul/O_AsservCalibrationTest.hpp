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
            FunctionalTest("Asserv_Calibration", "Calibration asserv par etapes", "cal")
    {
    }

    std::string defaultArgs() const override { return "0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <step> (0..5)\n"
            "        ex:   cal 0   # set pos puis lit pos+codeurs en boucle\n"
            "              cal 1   # codeurs seuls (pousser le robot a la main)\n"
            "              cal 2   # moteurs L+R 25% pendant ~5s\n"
            "              cal 3   # assistedHandling boucle (regler P)\n"
            "              cal 4   # line(100mm) en assisted (regler D translation)\n"
            "              cal 5   # rotateDeg(90) en assisted (regler D rotation)";
    }

    virtual ~O_AsservCalibrationTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
