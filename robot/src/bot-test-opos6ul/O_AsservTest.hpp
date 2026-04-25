/*!
 * \file
 * \brief Déclaration de la classe O_AsservTest.
 */

#ifndef O_ASSERVTEST_HPP
#define O_ASSERVTEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test de l'asservissement : déplacement vers plusieurs positions XY.
 *
 * Permet de tester les déplacements vers 3 points successifs (x,y), (x2,y2), (x3,y3)
 * avec détection d'obstacles et actionneurs AX12.
 */
class O_AsservTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_AsservTest");
        return instance;
    }

public:

    O_AsservTest() :
            FunctionalTest("Asserv run test", "go to different positions", "go")
    {
    }

    std::string defaultArgs() const override { return "40 500 500 /+ 200 200 0"; }

    std::string usageHelp() const override
    {
        return
            "        args: <speed%> <x> <y> [x2 y2] [x3 y3]   (jusqu'a 3 destinations)\n"
            "        opts: /+ x y a (pos initiale)\n"
            "        ex:   go 40 500 500 /+ 200 200 0           # vise (500,500) a 40%\n"
            "              go 40 500 500 800 500 /+ 200 200 0   # 2 destinations";
    }

    virtual ~O_AsservTest()
    {
    }

    virtual void run(int argc, char** argv);

    virtual void configureConsoleArgs(int argc, char** argv);

};

#endif
