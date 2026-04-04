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
            FunctionalTest("Asserv_Square", "effectue un carre")
    {
    }

    virtual ~O_Asserv_SquareTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
