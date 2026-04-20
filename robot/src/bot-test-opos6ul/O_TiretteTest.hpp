/*!
 * \file
 * \brief Définition de la classe O_TiretteTest.
 */

#ifndef OPOS6UL_TIRETTETEST_HPP
#define	OPOS6UL_TIRETTETEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Effectue un test sur les buttons.
 */
class O_TiretteTest: public FunctionalTest {
private:

    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref O_TiretteTest.
     */
    static inline const logs::Logger & logger() {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_TiretteTest");
        return instance;
    }
public:

    /*!
     * \brief Constructeur de la classe.
     */
    O_TiretteTest() :
            FunctionalTest("Tirette", "Tester la tirette.", "tir")
    {
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~O_TiretteTest() {
    }

    virtual void configureConsoleArgs(int argc, char** argv);

    std::string defaultArgs() const override { return "10"; }

    /*!
     * \brief Execute le test.
     */
    virtual void run(int argc, char** argv);

};

#endif
