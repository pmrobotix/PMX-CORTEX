/*!
 * \file
 * \brief Déclaration de la classe O_Asserv_CalageTest.
 */

#ifndef O_ASSERV_CALAGETEST_HPP
#define O_ASSERV_CALAGETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de recalage par bordure ou capteurs latéraux.
 *
 * Types de calage : B (bordure), R (capteur droit), L (capteur gauche),
 * DR (demo droit), DL (demo gauche).
 */
class O_Asserv_CalageTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_Asserv_CalageTest");
        return instance;
    }

public:

    O_Asserv_CalageTest() :
            FunctionalTest("Asserv recalage", "calage arriere")
    {
    }

    virtual ~O_Asserv_CalageTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
