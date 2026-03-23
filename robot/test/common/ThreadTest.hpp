/*!
 * \file
 * \brief Définition de la classe ThreadTest.
 */

#ifndef TEST_THREAD_TEST_HPP
#define TEST_THREAD_TEST_HPP

#include "../suite/UnitTest.hpp"
#include "../../src/common/thread/Thread.hpp"

namespace test {

/*!
 * \brief Thread de test simple qui incrémente un compteur.
 */
class SimpleThread : public utils::Thread {
private:
    int counter_;
public:
    SimpleThread() : counter_(0) {}
    virtual ~SimpleThread() {}

    /*!
     * \brief Exécute 100 incrémentations du compteur.
     */
    virtual void execute();

    /*!
     * \brief Retourne la valeur courante du compteur.
     */
    int counter() { return counter_; }
};

/*!
 * \brief Teste la classe utils::Thread.
 */
class ThreadTest : public UnitTest {
public:
    ThreadTest() : UnitTest("ThreadTest") {}
    virtual ~ThreadTest() {}

    virtual void suite();

    /*!
     * \brief Vérifie le démarrage, l'exécution et l'attente de fin d'un thread.
     */
    void testStartAndWait();

    /*!
     * \brief Vérifie les transitions d'état CREATED → STARTED → STOPPED.
     */
    void testState();

    /*!
     * \brief Vérifie la précision de sleep_for_millis.
     */
    void testSleepFunctions();
};

}

#endif
