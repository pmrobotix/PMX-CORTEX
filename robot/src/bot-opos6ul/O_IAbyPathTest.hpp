/*!
 * \file
 * \brief Definition de la classe O_IAByPathTest.
 */

#ifndef OPOS6UL_IABYPATHTEST_HPP_
#define	OPOS6UL_IABYPATHTEST_HPP_

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

class Playground;

/*!
 * \brief Effectue un test de l'IAByPath.
 */
class O_IAByPathTest: public FunctionalTest
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_IAByPathTest");
        return instance;
    }

    /*!
     * \brief Terrain de jeu pour le pathfinding (A*).
     */
    Playground *p_;

public:

    /*!
     * \brief Constructeur de la classe.
     */
    O_IAByPathTest() :
            FunctionalTest("IAbyPath", "test l'ia demo IAByPath")
    {
        p_ = NULL;
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~O_IAByPathTest()
    {
    }

    /*!
     * \brief Execute le test.
     */
    virtual void run(int argc, char** argv);

    /*!
     * \brief Configure les activités de l'IA pour le test.
     */
    void IASetup();

    /*!
     * \brief Initialise le terrain de jeu (grille de navigation A*).
     */
    void initPlayground();
};

#endif
