/*!
 * \file
 * \brief Declaration de la classe O_AsservWaypointTest.
 */

#ifndef O_ASSERVWAYPOINTTEST_HPP
#define O_ASSERVWAYPOINTTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de navigation par waypoints via Navigator.
 *
 * Permet de tester manualPath avec les 3 PathMode (STOP, CHAIN, CHAIN_NONSTOP).
 *
 * Options : /m 0=STOP 1=CHAIN 2=CHAIN_NONSTOP  /v vitesse%  /B detection  /+ x y a
 *
 * SVG :
 *   STOP           => pas de trait, robot vert a chaque waypoint
 *   CHAIN          => trait vert continu, robot vert a la fin
 *   CHAIN_NONSTOP  => trait vert pointille, robot vert a la fin
 *
 * === Carre 500mm en waypoints (depart 250,250) ===
 *
 *   # Mode STOP : pas de trait, robot vert a chaque coin
 *   ./bot-opos6ul wp 750 250  750 750  250 750  250 250 /m 0 /+ 250 250 0
 *
 *   # Mode CHAIN : trait vert continu, robot vert a la fin
 *   ./bot-opos6ul wp 750 250  750 750  250 750  250 250 /m 1 /+ 250 250 0
 *
 *   # Mode CHAIN_NONSTOP : trait vert pointille, robot vert a la fin
 *   ./bot-opos6ul wp 750 250  750 750  250 750  250 250 /m 2 /+ 250 250 0
 *
 * === Triangle 500mm ===
 *
 *   ./bot-opos6ul wp 750 250  500 683  250 250 /m 0 /+ 250 250 0
 *   ./bot-opos6ul wp 750 250  500 683  250 250 /m 2 /+ 250 250 0
 *
 * === Aller-retour ===
 *
 *   ./bot-opos6ul wp 750 250  250 250 /m 0 /+ 250 250 0
 *   ./bot-opos6ul wp 750 250  250 250 /m 2 /+ 250 250 0
 *
 * === Avec detection adverse ===
 *
 *   ./bot-opos6ul wp 750 250  750 750  250 750  250 250 /m 2 /B 1 /+ 250 250 0
 *
 * === Vitesse reduite 20% ===
 *
 *   ./bot-opos6ul wp 750 250  750 750  250 750  250 250 /m 2 /v 20 /+ 250 250 0
 *
 * === Vrai robot (ARM) — carre 200mm, vitesse 20% ===
 *
 *   # Mode STOP (prudent, premier test)
 *   ./bot-opos6ul wp 300 100  300 300  100 300  100 100 /m 0 /v 20 /+ 100 100 0
 *
 *   # Mode CHAIN
 *   ./bot-opos6ul wp 300 100  300 300  100 300  100 100 /m 1 /v 20 /+ 100 100 0
 *
 *   # Mode CHAIN_NONSTOP (trajectoire fluide)
 *   ./bot-opos6ul wp 300 100  300 300  100 300  100 100 /m 2 /v 20 /+ 100 100 0
 */
class O_AsservWaypointTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservWaypointTest");
        return instance;
    }

public:

    O_AsservWaypointTest() :
            FunctionalTest("Asserv_Waypoint", "Navigation par waypoints (Navigator).", "wp")
    {
    }

    std::string defaultArgs() const override { return "750 250 750 750 250 750 250 250 /m 0 /+ 250 250 0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <x> <y> (jusqu'a 5 waypoints x2 y2 / x3 y3 / x4 y4 / x5 y5)\n"
            "        opts: /m 0=STOP|1=CHAIN|2=CHAIN_NONSTOP  /v vit%  /B 0|1 detect\n"
            "              /+ x y a (pos initiale)\n"
            "        ex:   wp 750 250 /+ 250 250 0                                # 1 waypoint\n"
            "              wp 750 250 750 750 250 750 250 250 /m 0 /+ 250 250 0   # carre stop\n"
            "              wp 300 100 300 300 100 300 100 100 /m 2 /v 20 /+ 100 100 0  # ARM";
    }

    virtual ~O_AsservWaypointTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
