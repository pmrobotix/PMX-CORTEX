/*!
 * \file
 * \brief Définition de la classe Thread.
 */

#ifndef UTILS_THREAD_HPP
#define UTILS_THREAD_HPP

#include <pthread.h>
#include <string>
#include <thread>
#include "Mutex.hpp"
#include <chrono>

namespace utils {
/*!
 * \brief Enumération des états des threads.
 */
enum ThreadState {
    CREATED, STARTING, STARTED, STOPPED
};

typedef pthread_t ThreadId;

/*!
 * \brief Définit la priorité temps-réel (SCHED_FIFO) du thread spécifié.
 * \param p Priorité (0 = aucune, 1-99 = SCHED_FIFO, 99 = max).
 * \param name Nom du thread (pour les logs d'erreur).
 * \param this_thread ID du thread à modifier (par défaut : thread courant).
 * \return La priorité effectivement appliquée, ou négatif en cas d'erreur.
 */
int set_realtime_priority(int p = 0, std::string name = "", ThreadId this_thread = pthread_self());

/*!
 * \brief Met en pause le thread courant pendant \a usec microsecondes.
 */
void sleep_for_micros(int64_t usec);

/*!
 * \brief Met en pause le thread courant pendant \a msec millisecondes.
 */
void sleep_for_millis(int64_t msec);

/*!
 * \brief Met en pause le thread courant pendant \a sec secondes.
 */
void sleep_for_secs(int64_t sec);

/*!
 * \brief Classe de base pour les threads (wrapper POSIX pthread).
 *
 * Sous-classer et implémenter execute() pour définir le traitement du thread.
 * Appeler start() pour lancer, waitForEnd() pour attendre la fin (join).
 */
class Thread: public utils::Mutex {
protected:

    /*!
     * \brief Cette méthode sert de point d'entrée pour l'appel à
     * pcreate_thread.
     *
     * \param object
     *        Instance de Thread qui doit être lancé.
     */
    static void* entryPoint(void *object);

private:

    /*!
     * \brief Identifiant du thread lié.
     */
    ThreadId threadId_;

    /*!
     * \brief l'état du thread.
     */
    ThreadState state_;

    int priority_;

    std::string name_;

protected:

    /*!
     * \brief Constructeur de la classe.
     */
    Thread();

    /*!
     * \brief Donne la main à un autre thread (appelle pthread_yield).
     */
    void yield();

    /*!
     * \brief Donne la main à un autre thread de même priorité (appelle ::sched_yield).
     */
    void sched_yield();

    /*!
     * \brief Met en pause le thread pendant \a usec microsecondes.
     */
    void sleep_for_micros(int64_t usec);

    /*!
     * \brief Met en pause le thread pendant \a msec millisecondes.
     */
    void sleep_for_millis(int64_t msec);

    /*!
     * \brief Met en pause le thread pendant \a sec secondes.
     */
    void sleep_for_secs(int64_t sec);

    /*!
     * \brief Méthode virtuelle pure à implémenter dans les sous-classes.
     *
     * Contient le traitement exécuté par le thread.
     */
    virtual void execute() = 0;

public:

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~Thread() {
    }

    /*!
     * \brief Lance le thread.
     *
     * Initialise le thread POSIX et appelle execute() dans le nouveau thread.
     *
     * \param name Nom du thread (pour identification et logs).
     * \param priority Priorité SCHED_FIFO (0 = aucune, 1-99 = temps-réel).
     * \return \c true si le thread a démarré, \c false en cas d'erreur.
     */
    bool start(std::string name, int priority = 0);

    /*!
     * \brief Retourne l'état courant du thread.
     * \return L'état du thread (CREATED, STARTING, STARTED, STOPPED).
     */
    inline ThreadState state() {
        lock();
        ThreadState s = state_;
        unlock();
        return s;
    }

    /*!
     * \brief Modifie l'état du thread (thread-safe).
     * \param state Nouvel état.
     */
    inline void setState(ThreadState state) {
        lock();
        state_ = state;
        unlock();
    }

    /*!
     * \brief Indique si le thread est terminé ou pas encore démarré.
     * \return \c true si l'état est STOPPED ou CREATED.
     */
    inline bool isFinished() {
        return (state() == utils::STOPPED || state() == utils::CREATED);
    }

    /*!
     * \brief Attend la fin du thread (appelle pthread_join).
     */
    void waitForEnd() {
        std::this_thread::yield();
        pthread_join(threadId_, NULL);
    }

    /*!
     * \brief Annule le thread (appelle pthread_cancel).
     */
    void cancel() {
        pthread_cancel(threadId_);
    }

    /*!
     * \brief Retourne le nom du thread.
     */
    std::string name() {
        return name_;
    }

    /*!
     * \brief Retourne l'identifiant POSIX du thread.
     */
    ThreadId id() {
        return threadId_;
    }

};

}

#endif
