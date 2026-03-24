/*!
 * \file
 * \brief Définition de la classe PointerListTest.
 */

#ifndef TEST_POINTER_LIST_TEST_HPP
#define TEST_POINTER_LIST_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe utils::PointerList.
 */
class PointerListTest : public UnitTest {
public:
    PointerListTest() : UnitTest("PointerListTest") {}
    virtual ~PointerListTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie qu'une liste vide a une taille de 0.
     */
    void testEmptyList();

    /*!
     * \brief Vérifie l'ajout et l'itération sur des éléments.
     */
    void testAddAndIterate();

    /*!
     * \brief Vérifie la taille après ajouts multiples.
     */
    void testSize();

    /*!
     * \brief Vérifie la construction avec N copies.
     */
    void testConstructWithCopies();
};

}

#endif
