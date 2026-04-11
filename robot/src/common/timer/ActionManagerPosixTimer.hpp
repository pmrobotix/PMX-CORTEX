/*!
 * \file
 * \brief Definition de la classe ActionManagerPosixTimer (legacy, deprecated).
 *
 * Conserve uniquement pour les tests historiques.
 * Nouveau code : utiliser ActionTimerScheduler + ITimerScheduledListener.
 */

#ifndef COMMON_ACTIONMANAGERPOSIXTIMER_HPP_
#define COMMON_ACTIONMANAGERPOSIXTIMER_HPP_

#include <semaphore.h>
#include <atomic>
#include <list>
#include <string>

#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"
#include "utils/Chronometer.hpp"
#include "utils/PointerList.hpp"
#include "action/IAction.hpp"
#include "ITimerListener.hpp"
#include "ITimerPosixListener.hpp"

// Bruit interne tautologique : la classe deprecated reference l'interface deprecated.
// On suppresse ici uniquement, les consommateurs externes continuent de warner.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/*!
 * \brief Classe de gestion des actions du robot et des actions par timer.
 *        DEPRECATED — utiliser ActionTimerScheduler.
 */
class [[deprecated("Utiliser ActionTimerScheduler a la place")]] ActionManagerPosixTimer: public utils::Thread
{
private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("ActionManagerPosixTimer");
        return instance;
    }

    /*!
     * \brief Liste des actions a executer.
     */
    utils::PointerList<IAction*> actions_;

    /*!
     * \brief Liste des timers (gestion par timings), prend 100% processor.
     */
    utils::PointerList<ITimerListener*> timers_;

    /*!
     * \brief Liste des timers posix en cours d'execution.
     */
    utils::PointerList<ITimerPosixListener*> ptimers_;
    /*!
     * \brief Liste des timers posix a demarrer au plus vite.
     */
    utils::PointerList<ITimerPosixListener*> ptimers_tobestarted_;

    /*!
     * \brief Vaut true si les actions et timers doivent s'arreter.
     */
    std::atomic<bool> stopActionsAndTimers_;

    /*!
     * \brief Vaut true si les actions doivent se mettre en pause.
     */
    std::atomic<bool> pause_;

    /*!
     * \brief Chronometre lie au Minuteur.
     */
    utils::Chronometer chronoTimer_;

    /*!
     * \brief Mutex lie aux timers.
     */
    utils::Mutex mtimer_;
    /*!
     * \brief Mutex lie aux actions.
     */
    utils::Mutex maction_;

    /*!
     * \brief Semaphore de gestion de la boucle des actions.
     */
    sem_t AMT;
    /*!
     * \brief Mutex pour proteger les operations sur le semaphore.
     */
    utils::Mutex msem_;

protected:

    /*!
     * \brief Execute l'ensemble des actions enregistrees.
     */
    virtual void execute();

    void unblock(std::string debug = "ActionManagerPosixTimer")
    {
        msem_.lock();
        int val = 0;
        sem_getvalue(&AMT, &val);
        if (val > 1)
            logger().error() << "ERROR - la valeur de semaphore est > 1 !!" << val << logs::end;
        //dans le cas d'une attente, on debloque.
        if (val == 0)
            sem_post(&AMT);
        msem_.unlock();
    }

public:

    /*!
     * \brief Constructeur de la classe.
     */
    ActionManagerPosixTimer();

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~ActionManagerPosixTimer()
    {
        sem_destroy(&AMT);
    }

    /*!
     * \brief Retourne le nombre d'actions.
     */
    int countActions()
    {
        maction_.lock();
        int size = this->actions_.size();
        maction_.unlock();
        return size;
    }

    /*!
     * \brief Retourne le nombre de timers.
     */
    int countTimers()
    {
        mtimer_.lock();
        int size = this->timers_.size();
        mtimer_.unlock();
        return size;
    }
    /*!
     * \brief Retourne le nombre de timers posix.
     */
    int countPTimers()
    {
        mtimer_.lock();
        int size = this->ptimers_.size();
        mtimer_.unlock();
        return size;
    }

    /*!
     * \brief Ajout d'une action.
     * \param action L'action a ajouter.
     */
    void addAction(IAction *action)
    {
        maction_.lock();
        actions_.push_back(action);
        maction_.unlock();
        unblock("addAction");
    }

    /*!
     * \brief Ajout d'un timer. deprecated.
     * \param timer le timer a ajouter.
     */
    void addTimer(ITimerListener *timer)
    {
        if (timer->timeSpan() != 0) {
            mtimer_.lock();
            timers_.push_back(timer);
            mtimer_.unlock();
            unblock("addTimer");
        }
    }
    /*!
     * \brief Ajout d'un timer posix.
     * \param timer le timer posix a ajouter.
     */
    void addTimer(ITimerPosixListener *timer)
    {
        if (timer->timeSpan_us() != 0) {
            mtimer_.lock();
            ptimers_tobestarted_.push_back(timer);
            mtimer_.unlock();

            //give the signal to unblock the actiontimermanager
            unblock("addTimerPosix");

        } else {
            logger().error() << "timeSpan_us is 0 !!" << logs::end;
        }
    }

    /*!
     * \brief Arrete un timer specifique. Permet d'executer son action de fin puis le supprime de la liste.
     * \param timerNameToDelete Le label du timer.
     */
    void stopTimer(std::string timerNameToDelete);
    /*!
     * \brief Arrete un timer posix specifique. Permet d'executer son action de fin puis le supprime de la liste.
     * \param timerNameToDelete Le label du timer.
     */
    void stopPTimer(std::string timerNameToDelete);

    /*!
     * \brief Trouve un timer posix specifique.
     * \param timerNameToFind Le label du timer.
     */
    bool findPTimer(std::string timerNameToFind);

    /*!
     * \brief Arrete tous les timers posix. Permet d'executer leur action de fin puis les supprime de la liste.
     */
    void stopAllPTimers();

    /*!
     * \brief Vide la liste des actions actuellement enregistrees.
     */
    void clearActions()
    {
        maction_.lock();
        actions_.clear();
        maction_.unlock();
    }

    /*!
     * \brief Vide la liste des timers actuellement enregistres.
     */
    void clearTimers()
    {
        mtimer_.lock();
        timers_.clear();
        ptimers_.clear();
        ptimers_tobestarted_.clear();
        mtimer_.unlock();
    }

    /*!
     * \brief Signale au thread qu'il doit s'arreter (proprement).
     * Arrete aussi tous les timers posix en cours en lancant leur tache de fin.
     */
    void stop();

    /*!
     * \brief Met en pause la boucle du manager d'actions et de timer.
     */
    void pause(bool value);

    /*!
     * \brief Affiche via le logger les actions en cours.
     */
    void debugActions();

    /*!
     * \brief Affiche via le logger les timers posix en cours et en attente.
     */
    void debugPTimers();

    /*!
     * \brief Affiche via le logger les timers classiques en cours. Deprecated.
     */
    void debugTimers();
};

#pragma GCC diagnostic pop

#endif
