/*!
 * \file
 * \brief Définition de la classe ChronometerTest.
 */

#ifndef TEST_CHRONOMETER_TEST_HPP
#define TEST_CHRONOMETER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Teste la classe utils::Chronometer.
 */
class ChronometerTest : public UnitTest {
public:
    ChronometerTest() : UnitTest("ChronometerTest") {}
    virtual ~ChronometerTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie les transitions start/stop et le flag started().
     */
    void testStartStop();

    /*!
     * \brief Vérifie la mesure du temps écoulé après un sleep.
     */
    void testElapsedTime();

    /*!
     * \brief Vérifie le fonctionnement de setTimer/waitTimer.
     */
    void testSetTimerAndWait();
};

}

#endif
