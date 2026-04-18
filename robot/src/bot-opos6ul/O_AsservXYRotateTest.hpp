/*!
 * \file
 * \brief Déclaration de la classe O_AsservXYRotateTest.
 */

#ifndef O_ASSERVXYROTATETEST_HPP
#define O_ASSERVXYROTATETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de l'asservissement par coordonnées absolues XY et rotation.
 *
 * Déplace le robot vers des coordonnées (x, y) puis effectue une rotation,
 * avec gestion des obstacles avant/arrière.
 */
class O_AsservXYRotateTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservXYRotateTest");
        return instance;
    }

public:

    O_AsservXYRotateTest() :
            FunctionalTest("AsservXYRotate", "Effectue une liste de point par coordonnees", "xy")
    {
    }

    std::string defaultArgs() const override { return "40 500 500 500 0 0 1 /+ 250 250 0"; }

    virtual ~O_AsservXYRotateTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
