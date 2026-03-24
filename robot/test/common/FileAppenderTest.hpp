/*!
 * \file
 * \brief Définition de la classe FileAppenderTest.
 */

#ifndef TEST_FILEAPPENDER_TEST_HPP
#define TEST_FILEAPPENDER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe logs::FileAppender.
 */
class FileAppenderTest : public UnitTest {
public:
    /*!
     * \brief Constructeur.
     */
    FileAppenderTest() : UnitTest("FileAppenderTest") {}
    virtual ~FileAppenderTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie qu'un message écrit est bien présent dans le fichier après flush.
     */
    void testWriteAndFlush();

    /*!
     * \brief Vérifie que plusieurs messages sont écrits dans l'ordre.
     */
    void testMultipleMessages();

    /*!
     * \brief Vérifie que le fichier est vide si aucun message n'est écrit.
     */
    void testEmptyFile();

    /*!
     * \brief Vérifie que le destructeur flush automatiquement.
     */
    void testDestructorFlushes();
};

}

#endif
