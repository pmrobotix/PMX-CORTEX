/*!
 * \file
 * \brief Déclaration de la classe O_GroveColorTest.
 */

#ifndef O_GROVECOLORTEST_HPP
#define O_GROVECOLORTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test du capteur couleur Grove.
 */
class O_GroveColorTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_GroveColorTest");
        return instance;
    }

public:

    O_GroveColorTest() :
            FunctionalTest("Grove Color", "Tester le capteur coleur")
    {
    }

    virtual ~O_GroveColorTest()
    {
    }

    virtual void run(int argc, char** argv);

};

#endif
