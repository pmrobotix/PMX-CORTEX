/*!
 * \file
 * \brief Déclaration de la classe O_AsservLineRotateTest.
 */

#ifndef O_ASSERVLINEROTATETEST_HPP
#define O_ASSERVLINEROTATETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de l'asservissement : ligne droite et rotation.
 *
 * Permet de tester jusqu'à 4 segments successifs (distance + angle),
 * avec choix du mode (relatif/absolu), du pathfinding et de la vitesse.
 */
class O_AsservLineRotateTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservLineRotateTest");
        return instance;
    }

public:

    O_AsservLineRotateTest() :
            FunctionalTest("Asserv_LineRotate", "Effectue une ligne droite et une rotation eventuelle.")
    {
    }

    virtual ~O_AsservLineRotateTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
