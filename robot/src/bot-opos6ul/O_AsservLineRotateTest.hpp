/*!
 * \file
 * \brief Déclaration de la classe O_AsservLineRotateTest.
 */

#ifndef O_ASSERVLINEROTATETEST_HPP
#define O_ASSERVLINEROTATETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test ligne droite + rotation via Navigator.
 *
 * Permet de tester jusqu'a 5 segments successifs (distance + angle + back),
 * avec choix du mode (relatif/absolu), du Navigator, de la vitesse
 * et de la RetryPolicy.
 *
 * Options : /m 0=relatif(defaut) 1=absolu  /s vitesse%  /r repetitions
 *           /p 0=asserv direct 1=Navigator line  /B detection  /+ x y a
 *
 * === SIMU — carre 500mm ===
 *
 *   --- Mode asserv direct (/p 0, defaut) ---
 *
 *   # Carre 500mm, rotation relative 90
 *   ./bot-opos6ul t /n 8 500 90 0  500 90 0  500 90 0  500 90 0 /+ 250 250 0
 *
 *   # Carre 500mm, rotation absolue
 *   ./bot-opos6ul t /n 8 500 90 0  500 180 0  500 270 0  500 0 0 /m 1 /+ 250 250 0
 *
 *   # Aller-retour 500mm, 5 repetitions
 *   ./bot-opos6ul t /n 8 500 180 0 /r 5 /+ 250 250 0
 *
 *   # Triangle equilateral 500mm
 *   ./bot-opos6ul t /n 8 500 120 0  500 120 0  500 120 0 /+ 250 250 0
 *
 *   # Ligne 500mm marche arriere
 *   ./bot-opos6ul t /n 8 500 -1 1 /+ 500 500 0
 *
 *   --- Mode Navigator (/p 1) ---
 *
 *   # Carre 500mm avec retry
 *   ./bot-opos6ul t /n 8 500 90 0  500 90 0  500 90 0  500 90 0 /p 1 /+ 250 250 0
 *
 *   # Carre 500mm avec retry et detection adverse
 *   ./bot-opos6ul t /n 8 500 90 0  500 90 0  500 90 0  500 90 0 /p 1 /B 1 /+ 250 250 0
 *
 *   # Aller-retour 500mm avec retry, 5 repetitions
 *   ./bot-opos6ul t /n 8 500 180 0 /p 1 /r 5 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   # Asserv direct
 *   ./bot-opos6ul t /n 8 200 90 0  200 90 0  200 90 0  200 90 0 /s 20 /+ 100 100 0
 *
 *   # Navigator avec retry
 *   ./bot-opos6ul t /n 8 200 90 0  200 90 0  200 90 0  200 90 0 /p 1 /s 20 /+ 100 100 0
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
            FunctionalTest("Asserv_LineRotate", "Ligne droite + rotation (Navigator).")
    {
    }

    virtual ~O_AsservLineRotateTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
