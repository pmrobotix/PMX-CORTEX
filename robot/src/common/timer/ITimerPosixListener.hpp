/*!
 * \file
 * \brief Définition de l'interface ITimerPosixListener.
 *
 * Timer periodique base sur timer_create() POSIX.
 *
 * ## Historique : migration SIGEV_SIGNAL → SIGEV_THREAD (avril 2026)
 *
 * L'ancienne implementation utilisait SIGEV_SIGNAL + SIGALRM :
 * le timer envoyait un signal UNIX qui interrompait le thread principal
 * pour executer le callback onTimer() dans un signal handler.
 *
 * **Probleme** : le signal handler s'executait dans le contexte du thread
 * interrompu. Si ce thread tenait un mutex (msync_, SvgWriter, etc.),
 * le callback essayait de verrouiller le meme mutex → deadlock.
 * Ce bug causait des blocages aleatoires sur OPOS6UL (PMX-CORTEX)
 * et probablement aussi sur EV3 (PMX, Debian Linux).
 * Les mutex dans un signal handler sont de toute facon un comportement
 * indefini POSIX (non async-signal-safe).
 *
 * **Solution** : SIGEV_THREAD fait creer un vrai thread par glibc pour
 * chaque expiration du timer. Les mutex fonctionnent normalement entre
 * threads. Le surcout (~100us de pthread_create sur ARM Cortex-A7) est
 * negligeable pour des timers >= 10ms.
 *
 * References :
 * - https://man7.org/linux/man-pages/man2/timer_create.2.html
 * - https://man7.org/linux/man-pages/man7/sigevent.7.html
 * - https://man7.org/tlpi/code/online/dist/timers/ptmr_sigev_thread.c.html
 * - https://quirk.ch/2009/07/how-to-use-posix-timer-within-c-classes (implementation originale)
 *
 * LINKER : ajouter -lrt (timer_create/timer_settime ne sont pas dans la libc standard).
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

#include "../../common/utils/Chronometer.hpp"
#include "../../common/thread/Mutex.hpp"

static utils::Mutex mAlarm_;
static utils::Mutex mfct_;
/*!
 * \brief Cette interface représente une action executée par un timer lorsqu'il
 * atteint son seuil d'execution.
 */
class ITimerPosixListener
{
public:

    // Stored timer ID for alarm
    timer_t timerID;

    /*!
     * \brief Actions à executer pour le timer.
     */
    virtual void onTimer(utils::Chronometer chrono) = 0;

    /*!
     * \brief Actions de fin à executer pour le timer.
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

            // Pas de sigaction : SIGEV_THREAD cree un thread automatiquement

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

    inline bool getRunning()
    {
        return started_;
    }

    inline bool requestToStop()
    {
        return requestToStop_;
    }

    inline void setPause(bool paused)
    {
        paused_ = paused;
    }

    //deprecated
    inline void remove(timer_t thistimerID)
    {
        mfct_.lock();

        if (timer_delete(thistimerID) != 0) { // timer id koennte mit private probleme geben
            std::cout << "ERROR ITimerPosixListener remove - Could not delete the timerID: " << thistimerID
                    << std::endl;

        } else {
            started_ = false;
            requestToStop_ = false;
        }

        mfct_.unlock();
    }

    inline void remove()
    {
        mfct_.lock();

        if (timer_delete(this->timerID) != 0) { // timer id koennte mit private probleme geben
            std::cout << "ERROR ITimerPosixListener remove - Could not delete the timerID: " << this->timerID
                    << std::endl;
            //perror("ERROR ITimerPosixListener remove - Could not delete the timer");

        } else {
            started_ = false;
            requestToStop_ = false;
        }

        mfct_.unlock();
    }

    /**
     * Callback pour SIGEV_THREAD : executee dans un vrai thread separe.
     * Remplace l'ancien signal handler SIGALRM qui causait des deadlocks
     * (le signal interrompait le thread principal, meme s'il tenait un mutex).
     */
    static void timerThreadFunction(union sigval sv)
    {
        // tryLock : si le callback precedent tourne encore, on saute cette expiration.
        // Evite l'accumulation de threads (SIGEV_THREAD cree un thread par expiration)
        // qui causerait un OOM kill sur systeme embarque.
        if (!mAlarm_.tryLock()) {
            return;
        }

        ITimerPosixListener *ptrTimerPosix = reinterpret_cast<ITimerPosixListener*>(sv.sival_ptr);

        mfct_.lock();
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

        mAlarm_.unlock();
    }

    /*!
     * \brief Destructeur de la classe.
     */
    virtual inline ~ ITimerPosixListener()
    {
    }

protected:

    std::string name_;
    long timeSpan_us_;
    utils::Chronometer chrono;

    bool started_;
    bool paused_;
    bool requestToStop_;

    // The according signal event containing the this-pointer
    struct sigevent signalEvent;

    // The itimerspec structure for the timer
    struct itimerspec timerSpecs;

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

        // SIGEV_THREAD : le timer cree un vrai thread pour chaque expiration
        // (remplace SIGEV_SIGNAL/SIGALRM qui causait des deadlocks mutex)
        memset(&this->signalEvent, 0, sizeof(this->signalEvent));
        this->signalEvent.sigev_notify = SIGEV_THREAD;
        this->signalEvent.sigev_notify_function = &ITimerPosixListener::timerThreadFunction;
        this->signalEvent.sigev_notify_attributes = NULL;
        this->signalEvent.sigev_value.sival_ptr = (void*) this;

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
