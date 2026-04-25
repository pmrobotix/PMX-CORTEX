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
 * Options : /m 0=relatif(defaut) 1=absolu  /v vitesse%  /r repetitions
 *           /p 0=asserv direct 1=Navigator line  /B detection  /+ x y a
 *
 * === SIMU — carre 500mm ===
 *
 *   --- Mode asserv direct (/p 0, defaut) ---
 *
 *   # Carre 500mm, rotation relative 90
 *   ./bot-opos6ul lr 500 90 0  500 90 0  500 90 0  500 90 0 /+ 250 250 0
 *
 *   # Carre 500mm, rotation absolue
 *   ./bot-opos6ul lr 500 90 0  500 180 0  500 270 0  500 0 0 /m 1 /+ 250 250 0
 *
 *   # Aller-retour 500mm, 5 repetitions
 *   ./bot-opos6ul lr 500 180 0 /r 5 /+ 250 250 0
 *
 *   # Triangle equilateral 500mm
 *   ./bot-opos6ul lr 500 120 0  500 120 0  500 120 0 /+ 250 250 0
 *
 *   # Ligne 500mm marche arriere
 *   ./bot-opos6ul lr 500 -1 1 /+ 500 500 0
 *
 *   --- Mode Navigator (/p 1) ---
 *
 *   # Carre 500mm avec retry
 *   ./bot-opos6ul lr 500 90 0  500 90 0  500 90 0  500 90 0 /p 1 /+ 250 250 0
 *
 *   # Carre 500mm avec retry et detection adverse
 *   ./bot-opos6ul lr 500 90 0  500 90 0  500 90 0  500 90 0 /p 1 /B 1 /+ 250 250 0
 *
 *   # Aller-retour 500mm avec retry, 5 repetitions
 *   ./bot-opos6ul lr 500 180 0 /p 1 /r 5 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   # Asserv direct
 *   ./bot-opos6ul lr 200 90 0  200 90 0  200 90 0  200 90 0 /v 20 /+ 100 100 0
 *
 *   # Navigator avec retry
 *   ./bot-opos6ul lr 200 90 0  200 90 0  200 90 0  200 90 0 /p 1 /v 20 /+ 100 100 0
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
            FunctionalTest("Asserv_LineRotate", "Ligne droite + rotation (Navigator).", "lr")
    {
    }

    std::string defaultArgs() const override { return "500 90 0 500 90 0 500 90 0 500 90 0 /+ 250 250 0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <d> [a] [back] (jusqu'a 5 segments d2/a2/back2 ...)\n"
            "        opts: /m 0=relatif|1=absolu  /p 0=asserv|1=Navigator  /v vit%  /B 0|1 detect\n"
            "              /r repetitions  /+ x y a (pos initiale)\n"
            "        ex:   lr 30                       # avance 30mm (40% defaut)\n"
            "              lr 30 0 0 /B 0              # 30mm sans rot ni detection\n"
            "              lr 500 90 0 /+ 250 250 0    # 500mm + rot 90, depart (250,250)";
    }

    virtual ~O_AsservLineRotateTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
