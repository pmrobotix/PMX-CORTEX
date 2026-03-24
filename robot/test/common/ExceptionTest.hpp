/*!
 * \file
 * \brief Définition de la classe ExceptionTest.
 */

#ifndef TEST_EXCEPTION_TEST_HPP
#define TEST_EXCEPTION_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe logs::Exception.
 */
class ExceptionTest : public UnitTest {
public:
    ExceptionTest() : UnitTest("ExceptionTest") {}
    virtual ~ExceptionTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie la construction et l'accès au message.
     */
    void testMessageAccess();

    /*!
     * \brief Vérifie la modification du message.
     */
    void testMessageModification();

    /*!
     * \brief Vérifie what() retourne le bon message.
     */
    void testWhat();

    /*!
     * \brief Vérifie le throw/catch via std::exception.
     */
    void testThrowCatch();
};

}

#endif
