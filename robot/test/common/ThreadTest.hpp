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
 * \brief Thread qui incrémente un compteur partagé protégé par un Mutex.
 */
class MutexCounterThread : public utils::Thread {
private:
    utils::Mutex & mutex_;
    int & sharedCounter_;
    int iterations_;
public:
    MutexCounterThread(utils::Mutex & mutex, int & counter, int iterations)
        : mutex_(mutex), sharedCounter_(counter), iterations_(iterations) {}
    virtual ~MutexCounterThread() {}

    virtual void execute();
};

/*!
 * \brief Thread qui dort longtemps (pour tester cancel).
 */
class SleepyThread : public utils::Thread {
private:
    bool executed_;
public:
    SleepyThread() : executed_(false) {}
    virtual ~SleepyThread() {}

    virtual void execute();

    bool wasExecuted() { return executed_; }
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

    /*!
     * \brief Vérifie que le nom et l'ID du thread sont correctement assignés.
     */
    void testNameAndId();

    /*!
     * \brief Vérifie que plusieurs threads concurrents avec Mutex
     * produisent un résultat cohérent.
     */
    void testConcurrentMutex();

    /*!
     * \brief Vérifie que cancel() arrête un thread en cours.
     */
    void testCancel();

    /*!
     * \brief Vérifie la précision de sleep_for_micros.
     */
    void testSleepMicros();
};

}

#endif
