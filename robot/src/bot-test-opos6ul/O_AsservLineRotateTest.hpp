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
 * Permet de tester jusqu'a 5 segments successifs. Chaque segment = 4 params :
 *   aPre  : rotation AVANT la LINE (deg, 0 = pas de pre-rotation)
 *   d     : distance LINE en mm (-1 = segment skipe, 0 = pas de LINE)
 *   aPost : rotation APRES la LINE (deg, 0 = pas de post-rotation)
 *   back  : 0 = forward, 1 = backward (inverse le signe de d)
 *
 * → permet ROT-LINE-ROT, juste ROT, juste LINE, ou n'importe quel sous-ensemble.
 *
 * Options : /m 0=relatif(defaut) 1=absolu  /v vitesse%  /r repetitions
 *           /p 0=asserv direct 1=Navigator line  /B detection  /+ x y a
 *
 * === SIMU — carre 500mm ===
 *
 *   --- Mode asserv direct (/p 0, defaut) ---
 *
 *   # Carre 500mm : LINE 500 puis ROT 90, x4 (aPre=0, aPost=90)
 *   ./bot-opos6ul lr 0 500 90 0  0 500 90 0  0 500 90 0  0 500 90 0 /+ 250 250 0
 *
 *   # Carre 500mm en partant par la rotation : ROT 90 puis LINE 500, x4
 *   ./bot-opos6ul lr 90 500 0 0  90 500 0 0  90 500 0 0  90 500 0 0 /+ 250 250 0
 *
 *   # Carre 500mm, rotation absolue (cap 90, 180, 270, 0)
 *   ./bot-opos6ul lr 0 500 90 0  0 500 180 0  0 500 270 0  0 500 0 0 /m 1 /+ 250 250 0
 *
 *   # Aller-retour 500mm, 5 repetitions (LINE puis demi-tour)
 *   ./bot-opos6ul lr 0 500 180 0 /r 5 /+ 250 250 0
 *
 *   # Triangle equilateral 500mm
 *   ./bot-opos6ul lr 0 500 120 0  0 500 120 0  0 500 120 0 /+ 250 250 0
 *
 *   # Ligne 500mm marche arriere (back=1)
 *   ./bot-opos6ul lr 0 500 0 1 /+ 500 500 0
 *
 *   # Rotation pure 90 puis LINE 100 (test ROT-avant-LINE)
 *   ./bot-opos6ul lr 90 100 0 0 /+ 250 250 0
 *
 *   --- Mode Navigator (/p 1) ---
 *
 *   # Carre 500mm avec retry
 *   ./bot-opos6ul lr 0 500 90 0  0 500 90 0  0 500 90 0  0 500 90 0 /p 1 /+ 250 250 0
 *
 *   # Carre 500mm avec retry et detection adverse
 *   ./bot-opos6ul lr 0 500 90 0  0 500 90 0  0 500 90 0  0 500 90 0 /p 1 /B 1 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   # Asserv direct
 *   ./bot-opos6ul lr 0 200 90 0  0 200 90 0  0 200 90 0  0 200 90 0 /v 20 /+ 100 100 0
 *
 *   # Navigator avec retry
 *   ./bot-opos6ul lr 0 200 90 0  0 200 90 0  0 200 90 0  0 200 90 0 /p 1 /v 20 /+ 100 100 0
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

    std::string defaultArgs() const override { return "0 500 90 0 0 500 90 0 0 500 90 0 0 500 90 0 /+ 250 250 0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <aPre> <d> <aPost> <back>  (jusqu'a 5 segments)\n"
            "              aPre  : rotation AVANT LINE (deg, 0 = aucune)\n"
            "              d     : distance LINE mm  (-1 = skip segment, 0 = pas de LINE)\n"
            "              aPost : rotation APRES LINE (deg, 0 = aucune)\n"
            "              back  : 0=forward, 1=backward (inverse signe de d)\n"
            "        opts: /m 0=relatif|1=absolu  /p 0=asserv|1=Navigator  /v vit%  /B 0|1 detect\n"
            "              /r repetitions  /+ x y a (pos initiale)\n"
            "        ex:   lr 0 30 0 0                 # avance 30mm pur (40% defaut)\n"
            "              lr 0 500 90 0 /+ 250 250 0  # LINE 500 puis ROT 90\n"
            "              lr 90 100 0 0 /+ 250 250 0  # ROT 90 puis LINE 100\n"
            "              lr 90 100 -45 0             # ROT 90, LINE 100, ROT -45";
    }

    virtual ~O_AsservLineRotateTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
