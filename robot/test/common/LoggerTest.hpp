/*!
 * \file
 * \brief Définition de la classe LoggerTest.
 */

#ifndef TEST_LOGGER_TEST_HPP
#define TEST_LOGGER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe logs::Logger.
 */
class LoggerTest : public UnitTest {
public:
    LoggerTest() : UnitTest("LoggerTest") {}
    virtual ~LoggerTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie les messages simples (debug, info, warn, error).
     */
    void testSimpleMessages();

    /*!
     * \brief Vérifie les messages via l'API stream (<<).
     */
    void testStreamMessages();

    /*!
     * \brief Vérifie que les messages sous le niveau du logger sont filtrés.
     */
    void testLevelFiltering();

    /*!
     * \brief Vérifie le nom et le niveau du logger.
     */
    void testLoggerProperties();

    /*!
     * \brief Vérifie la construction par copie d'un logger parent.
     */
    void testLoggerCopyFromParent();
};

}

#endif
