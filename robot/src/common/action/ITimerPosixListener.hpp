/*!
 * \file
 * \brief Definition de l'interface ITimerPosixListener.
 *
 *  LINKER Add -lrt to your link command. timer_create and timer_settime are not part of the C Standard library.
 *  reference
 *  https://quirk.ch/2009/07/how-to-use-posix-timer-within-c-classes
 */

#ifndef COMMON_UTILS_ITIMERPOSIXLISTENER_HPP
#define COMMON_UTILS_ITIMERPOSIXLISTENER_HPP

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "utils/Chronometer.hpp"
#include "thread/Mutex.hpp"

static utils::Mutex mAlarm_;
static utils::Mutex mfct_;
/*!
 * \brief Cette interface represente une action executee par un timer lorsqu'il
 * atteint son seuil d'execution.
 */
class ITimerPosixListener
{
public:

    timer_t timerID;  ///< Identifiant du timer POSIX.

    /*!
     * \brief Actions a executer pour le timer.
     */
    virtual void onTimer(utils::Chronometer chrono) = 0;

    /*!
     * \brief Actions de fin a executer pour le timer.
     */
    virtual void onTimerEnd(utils::Chronometer chrono) = 0;

    /*!
     * \brief Getter sur les infos permettant d'identifier le timer.
     */
    virtual std::string name()
    {
        return name_;
    }

    /*!
     * \brief Getter sur le temps interval du timer.
     */
    inline int timeSpan_us()
    {
        return timeSpan_us_;
    }

    /*!
     * \brief Demarre le timer POSIX. Cree le timer systeme et installe le signal handler.
     */
    inline void startTimer()
    {
        mfct_.lock();
        if (!started_) {
            if (timer_create(CLOCK_REALTIME, &this->signalEvent, &this->timerID) != 0) {
                std::cout << "ERROR ITimerPosixListener Could not create the timer name:" << this->name() << std::endl;
                perror("Could not create the timer");
                sleep(1);
                exit(1);
            }
            if (sigaction(SIGALRM, &this->SignalAction, NULL)) {
                std::cout << "ERROR ITimerPosixListener Could not install new signal handler, id:" << this->timerID
                        << " name:" << this->name() << std::endl;
                perror("Could not install new signal handler");
                sleep(1);
                exit(1);
            }
            if (timer_settime(this->timerID, 0, &this->timerSpecs, NULL) == -1) {
                std::cout << "ERROR ITimerPosixListener Could not start timer, id:" << this->timerID << " name:"
                        << this->name() << std::endl;
                perror("Could not start timer!");
                sleep(1);
                exit(1);
            }

            if (!chrono.started())
                chrono.start();
            started_ = true;
        } else {
            std::cout << "ERROR ITimerPosixListener timer already started, id:" << this->timerID << " name:"
                    << this->name() << std::endl;
        }
        mfct_.unlock();
    }

    /*!
     * \brief Indique si le timer est demarre.
     * \return true si le timer est en cours d'execution.
     */
    inline bool getRunning()
    {
        return started_;
    }

    /*!
     * \brief Indique si le timer a demande son arret.
     * \return true si le timer doit etre arrete.
     */
    inline bool requestToStop()
    {
        return requestToStop_;
    }

    /*!
     * \brief Met en pause ou reprend le timer.
     * \param paused true pour mettre en pause, false pour reprendre.
     */
    inline void setPause(bool paused)
    {
        paused_ = paused;
    }

    /*!
     * \brief Supprime un timer POSIX par son identifiant. Deprecated, utiliser remove().
     * \param thistimerID Identifiant du timer a supprimer.
     */
    inline void remove(timer_t thistimerID)
    {
        mfct_.lock();

        if (timer_delete(thistimerID) != 0) {
            std::cout << "ERROR ITimerPosixListener remove - Could not delete the timerID: " << thistimerID
                    << std::endl;

        } else {
            started_ = false;
            requestToStop_ = false;
        }

        mfct_.unlock();
    }

    /*!
     * \brief Supprime le timer POSIX courant.
     */
    inline void remove()
    {
        mfct_.lock();

        if (timer_delete(this->timerID) != 0) {
            std::cout << "ERROR ITimerPosixListener remove - Could not delete the timerID: " << this->timerID
                    << std::endl;

        } else {
            started_ = false;
            requestToStop_ = false;
        }

        mfct_.unlock();
    }

    /*!
     * \brief Handler de signal POSIX appele a chaque expiration du timer.
     *
     * Recupere le pointeur vers l'instance ITimerPosixListener depuis
     * la structure siginfo et appelle onTimer() ou onTimerEnd().
     * \param sigNumb Numero du signal (SIGALRM).
     * \param si Informations du signal (contient le pointeur this).
     * \param uc Contexte utilisateur (non utilise).
     */
    static void alarmFunction(int sigNumb, siginfo_t *si, void *uc)
    {

        // lock mAlarm_
        mAlarm_.lock();

        mfct_.lock();
// get the pointer out of the siginfo structure and asign it to a new pointer variable
        ITimerPosixListener *ptrTimerPosix = reinterpret_cast<ITimerPosixListener*>(si->si_value.sival_ptr);
// call the member function
        if (!ptrTimerPosix->paused_ && ptrTimerPosix->started_) {
            ptrTimerPosix->onTimer(ptrTimerPosix->chrono);

        }
        mfct_.unlock();
        if (ptrTimerPosix->requestToStop()) {

            mfct_.lock();
            ptrTimerPosix->onTimerEnd(ptrTimerPosix->chrono);
            mfct_.unlock();

            ptrTimerPosix->remove(ptrTimerPosix->timerID);

        }

        // unlock
        mAlarm_.unlock();
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~ ITimerPosixListener()
    {
    }

protected:

    std::string name_;              ///< Nom identifiant le timer.
    long timeSpan_us_;              ///< Intervalle du timer en microsecondes.
    utils::Chronometer chrono;      ///< Chronometre interne du timer.

    bool started_;                  ///< true si le timer est demarre.
    bool paused_;                   ///< true si le timer est en pause.
    bool requestToStop_;            ///< true si le timer a demande son arret.

    sigset_t SigBlockSet;           ///< Ensemble de signaux bloques.
    struct sigevent signalEvent;    ///< Evenement signal contenant le pointeur this.
    struct sigaction SignalAction;   ///< Action associee au signal SIGALRM.
    struct itimerspec timerSpecs;    ///< Specification d'intervalle du timer POSIX.

    /*!
     * \brief Initialise le timer avec un label et un intervalle.
     * \param label Nom du timer.
     * \param time_us Intervalle en microsecondes.
     */
    void init(std::string label, uint time_us)
    {
        mfct_.lock();

        name_ = label;
        timeSpan_us_ = time_us;
        chrono = utils::Chronometer("ITimerListener-" + name_);
        started_ = false;
        paused_ = false;
        timerID = 0;
        requestToStop_ = false;

        // One second till first occurrence
        this->timerSpecs.it_value.tv_sec = 0;  // first execution time
        this->timerSpecs.it_value.tv_nsec = 500000;
        // and then all 3 seconds a timer alarm
        if (timeSpan_us_ >= 1000000)
            this->timerSpecs.it_interval.tv_sec = (int) (timeSpan_us_ / 1000000.0);
        else
            this->timerSpecs.it_interval.tv_sec = 0;
        this->timerSpecs.it_interval.tv_nsec = (int) ((timeSpan_us_ % 1000000) * 1000.0);

        // Clear the sa_mask
        sigemptyset(&this->SignalAction.sa_mask);
        // set the SA_SIGINFO flag to use the extended signal-handler function
        this->SignalAction.sa_flags = SA_SIGINFO;

        // Define sigaction method
        // This function will be called by the signal
        this->SignalAction.sa_sigaction = &ITimerPosixListener::alarmFunction;

        // Define sigEvent
        // This information will be forwarded to the signal-handler function
        memset(&this->signalEvent, 0, sizeof(this->signalEvent));
        // With the SIGEV_SIGNAL flag we say that there is sigev_value
        this->signalEvent.sigev_notify = SIGEV_SIGNAL;
        // Now it's possible to give a pointer to the object
        this->signalEvent.sigev_value.sival_ptr = (void*) this;
        // Declare this signal as Alarm Signal

        this->signalEvent.sigev_signo = SIGALRM;

        mfct_.unlock();

    }

    /*!
     * \brief Constructeur de la classe.
     */
    ITimerPosixListener() :
            chrono("ITimerPosixListener")
    {
        timerID = 0;
        started_ = false;
        paused_ = false;
        requestToStop_ = false;
        timeSpan_us_ = 0;
        name_ = "ITimerPosixListener_default";
    }
};

#endif
