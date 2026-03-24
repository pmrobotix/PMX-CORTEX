/*!
 * \file
 * \brief Définition de la classe LevelTest.
 */

#ifndef TEST_LEVEL_TEST_HPP
#define TEST_LEVEL_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe logs::Level.
 */
class LevelTest : public UnitTest {
public:
    LevelTest() : UnitTest("LevelTest") {}
    virtual ~LevelTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie les noms associés aux niveaux.
     */
    void testNames();

    /*!
     * \brief Vérifie l'opérateur >= entre niveaux.
     */
    void testGreaterOrEqual();

    /*!
     * \brief Vérifie l'opérateur == entre niveaux.
     */
    void testEquality();

    /*!
     * \brief Vérifie l'opérateur != entre niveaux.
     */
    void testInequality();

    /*!
     * \brief Vérifie l'ordre des niveaux : ALL < DEBUG < INFO < WARN < TELEM < ERROR < NONE.
     */
    void testOrdering();
};

}

#endif
