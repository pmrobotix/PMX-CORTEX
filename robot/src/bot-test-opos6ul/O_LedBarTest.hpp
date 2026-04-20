/*!
 * \file
 * \brief Definition de la classe O_LedBarTest.
 */

#ifndef OPOS6UL_LEDBARTEST_HPP
#define	OPOS6UL_LEDBARTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Effectue un test de clignotement des LEDs du tableau d'affichage.
 */
class O_LedBarTest: public FunctionalTest {
private:

    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_LedBarTest");
        return instance;
    }
public:

    O_LedBarTest() :
            FunctionalTest("LedBar", "Blink Leds", "led")
    {
    }

    virtual ~O_LedBarTest() {
    }

    virtual void run(int argc, char** argv);
};

#endif
