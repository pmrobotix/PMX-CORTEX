/*!
 * \file
 * \brief Déclaration de la classe O_Asserv_SquareTest.
 */

#ifndef O_ASSERV_SQUARETEST_HPP
#define O_ASSERV_SQUARETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de l'asservissement : parcours en carré.
 *
 * Effectue un carré de côté d à partir de la position (x, y),
 * en utilisant moveForwardTo pour chaque sommet.
 */
class O_Asserv_SquareTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_Asserv_SquareTest");
        return instance;
    }

public:

    O_Asserv_SquareTest() :
            FunctionalTest("Asserv_Square", "effectue un carre", "sq")
    {
    }

    std::string defaultArgs() const override { return "250 250 0 500 1 0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <x> <y> <a> <d> [nb] [cw]\n"
            "              x,y = depart (mm), a = angle initial (deg), d = cote du carre (mm)\n"
            "              nb  = nombre de tours, cw = sens (0=CCW defaut, 1=CW)\n"
            "        ex:   sq 250 250 0 500             # carre 500mm depuis (250,250)\n"
            "              sq 100 100 0 200 2 1         # 2 tours sens horaire";
    }

    virtual ~O_Asserv_SquareTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
