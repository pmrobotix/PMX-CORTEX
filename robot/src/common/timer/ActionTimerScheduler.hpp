/*!
 * \file
 * \brief Definition de la classe ActionTimerScheduler.
 *
 * Scheduler unique : 1 SEUL thread persistant qui gere :
 *  - Toutes les IAction (taches asynchrones one-shot ou recurrentes)
 *  - Tous les ITimerScheduledListener (callbacks periodiques)
 *
 * Remplacement de ActionManagerTimer (legacy SIGEV_THREAD/SIGALRM) avec :
 *  - 1 seul thread total (pas N threads par timer)
 *  - Pas de signal handler (pas de deadlock mutex possible)
 *  - clock_nanosleep ABSTIME pour precision sans drift
 *  - API similaire (addAction, addTimer, stop, pause, find, debug)
 *
 * Contraintes :
 *  - Les IAction et onTimer() doivent etre courts (< 1ms typique). Une operation
 *    longue bloque tous les autres timers et actions pendant son execution.
 *  - Pour une operation longue : la decouper en plusieurs etapes (state machine).
 */

#ifndef COMMON_TIMER_ACTIONTIMERSCHEDULER_HPP_
#define COMMON_TIMER_ACTIONTIMERSCHEDULER_HPP_

#include <semaphore.h>
#include <atomic>
#include <list>
#include <string>

#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"
#include "utils/Chronometer.hpp"
#include "utils/PointerList.hpp"
#include "thread/Thread.hpp"
#include "thread/Mutex.hpp"
#include "action/IAction.hpp"
#include "ITimerScheduledListener.hpp"

class ActionTimerScheduler: public utils::Thread
{
private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("ActionTimerScheduler");
        return instance;
    }

    /*!
     * \brief Liste des actions a executer (FIFO).
     */
    utils::PointerList<IAction*> actions_;

    /*!
     * \brief Liste des timers actifs.
     */
    utils::PointerList<ITimerScheduledListener*> timers_;

    /*!
     * \brief Liste des timers a ajouter (transferes vers timers_ au prochain cycle).
     */
    utils::PointerList<ITimerScheduledListener*> timers_to_add_;

    std::atomic<bool> stop_;
    std::atomic<bool> pause_;

    utils::Chronometer chronoTimer_;

    utils::Mutex maction_;
    utils::Mutex mtimer_;

    /*!
     * \brief Semaphore pour reveiller le thread quand on ajoute action/timer
     *        ou quand on demande pause/stop.
     */
    sem_t sem_;
    utils::Mutex msem_;

protected:

    virtual void execute() override;

    /*!
     * \brief Reveille le thread s'il est en attente sur le semaphore.
     */
    void unblock(std::string debug = "ActionTimerScheduler")
    {
        msem_.lock();
        int val = 0;
        sem_getvalue(&sem_, &val);
        if (val == 0)
            sem_post(&sem_);
        msem_.unlock();
    }

public:

    ActionTimerScheduler();

    virtual ~ActionTimerScheduler()
    {
        sem_destroy(&sem_);
    }

    // ========== Actions ==========

    /*!
     * \brief Ajoute une action a executer.
     *        L'action est repoussee en queue tant que execute() retourne true.
     */
    void addAction(IAction *action)
    {
        if (action == nullptr) return;
        maction_.lock();
        actions_.push_back(action);
        maction_.unlock();
        unblock("addAction");
    }

    int countActions()
    {
        maction_.lock();
        int n = (int)actions_.size();
        maction_.unlock();
        return n;
    }

    void clearActions()
    {
        maction_.lock();
        actions_.clear();
        maction_.unlock();
    }

    // ========== Timers ==========

    /*!
     * \brief Ajoute un timer scheduled au scheduler.
     *        Le timer commencera a tick au prochain cycle de la boucle.
     */
    void addTimer(ITimerScheduledListener *timer)
    {
        if (timer == nullptr) {
            logger().error() << "addTimer: timer is NULL" << logs::end;
            return;
        }
        if (timer->timeSpan_us() == 0) {
            logger().error() << "addTimer: timeSpan_us is 0 for " << timer->name() << logs::end;
            return;
        }
        mtimer_.lock();
        timers_to_add_.push_back(timer);
        mtimer_.unlock();
        unblock("addTimer");
    }

    /*!
     * \brief Arrete un timer par son nom (appelle onTimerEnd puis le retire).
     */
    void stopTimer(std::string name);

    /*!
     * \brief Verifie qu'un timer est present (actifs ou en attente d'ajout).
     */
    bool findTimer(std::string name);

    /*!
     * \brief Arrete tous les timers (sans les supprimer ; useful pour stop global).
     */
    void stopAllTimers();

    int countTimers()
    {
        mtimer_.lock();
        int n = (int)timers_.size() + (int)timers_to_add_.size();
        mtimer_.unlock();
        return n;
    }

    void clearTimers()
    {
        mtimer_.lock();
        timers_.clear();
        timers_to_add_.clear();
        mtimer_.unlock();
    }

    // ========== Controle global ==========

    /*!
     * \brief Demande l'arret du thread (apres avoir arrete tous les timers).
     */
    void stop();

    /*!
     * \brief Met en pause / reprend le scheduler.
     */
    void pause(bool value)
    {
        pause_ = value;
        // Met aussi tous les timers en pause
        mtimer_.lock();
        for (auto i = timers_.begin(); i != timers_.end(); ++i) {
            if (*i != nullptr) (*i)->setPause(value);
        }
        mtimer_.unlock();
        unblock("pause");
    }

    // ========== Debug ==========

    void debugActions();
    void debugTimers();
};

#endif
