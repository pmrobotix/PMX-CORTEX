/*!
 * \file
 * \brief Définition de la classe SvgAppenderTest.
 */

#ifndef TEST_SVGAPPENDER_TEST_HPP
#define TEST_SVGAPPENDER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe logs::SvgAppender.
 */
class SvgAppenderTest : public UnitTest {
public:
    /*!
     * \brief Constructeur.
     */
    SvgAppenderTest() : UnitTest("SvgAppenderTest") {}
    virtual ~SvgAppenderTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie qu'un élément SVG est écrit dans le fichier après flush.
     */
    void testWriteAndFlush();

    /*!
     * \brief Vérifie que plusieurs éléments SVG sont écrits dans l'ordre.
     */
    void testMultipleElements();

    /*!
     * \brief Vérifie que le fichier est vide sans message.
     */
    void testEmptyFile();

    /*!
     * \brief Vérifie que le destructeur flush automatiquement.
     */
    void testDestructorFlushes();
};

}

#endif
