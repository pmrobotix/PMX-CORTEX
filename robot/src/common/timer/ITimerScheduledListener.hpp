/*!
 * \file
 * \brief Interface ITimerScheduledListener — callback periodique execute par
 *        ActionManagerTimerThread (1 thread partage pour tous les timers).
 *
 * Remplacement de ITimerThreadListener (1 thread par timer) et de
 * ITimerPosixListener (SIGEV_THREAD/SIGALRM) qui avaient chacun leurs problemes :
 *  - ITimerPosixListener : deadlock (signal handler + mutex) + OOM kill (SIGEV_THREAD)
 *  - ITimerThreadListener : 1 thread persistant par timer (peu scalable au-dela de ~10 timers)
 *
 * Approche retenue :
 *  - PAS d'heritage Thread : c'est juste un objet avec un callback
 *  - PAS de signal handler : tout dans des appels de fonction normaux
 *  - 1 SEUL thread (ActionManagerTimerThread) execute tous les timers en boucle
 *  - Le scheduler dort avec clock_nanosleep ABSTIME jusqu'au prochain tick
 *
 * Contraintes :
 *  - onTimer() doit etre COURT (typiquement < 1ms) car il bloque les autres timers
 *    et les actions pendant son execution.
 *  - Si une operation est longue (I2C, attente AX12), la decouper en plusieurs
 *    ticks ou la mettre dans une IAction asynchrone.
 *  - Mutex OK : on est dans un thread normal, pas un signal handler.
 */

#ifndef COMMON_TIMER_ITIMERSCHEDULEDLISTENER_HPP
#define COMMON_TIMER_ITIMERSCHEDULEDLISTENER_HPP

#include <atomic>
#include <cstdint>
#include <string>

#include "../utils/Chronometer.hpp"

/*!
 * \brief Interface pour un timer execute par ActionTimerScheduler.
 *
 * Sous-classer et implementer onTimer() / onTimerEnd().
 * Utiliser init(name, period_us) dans le constructeur.
 * Ajouter au scheduler via Actions::addTimer(this).
 */
class ITimerScheduledListener
{
    friend class ActionTimerScheduler; // acces a next_tick_us_ et flags

public:

    /*!
     * \brief Action executee a chaque tick du timer.
     *        Doit etre courte (< 1ms typiquement).
     */
    virtual void onTimer(utils::Chronometer chrono) = 0;

    /*!
     * \brief Action executee une fois apres l'arret du timer.
     */
    virtual void onTimerEnd(utils::Chronometer chrono) = 0;

    /*!
     * \brief Nom du timer (pour identification dans les logs et stop par nom).
     */
    virtual std::string name()
    {
        return name_;
    }

    /*!
     * \brief Periode du timer en microsecondes.
     */
    inline long timeSpan_us()
    {
        return period_us_;
    }

    /*!
     * \brief Met le timer en pause (n'est plus tick) ou le reactive.
     */
    inline void setPause(bool paused)
    {
        paused_ = paused;
    }

    /*!
     * \brief Demande l'arret propre du timer.
     *        Le scheduler appellera onTimerEnd() puis le retirera de la liste.
     */
    inline void requestStop()
    {
        requestToStop_ = true;
    }

    inline bool requestToStop()
    {
        return requestToStop_;
    }

    /*!
     * \brief Indique si le timer est en cours d'execution dans le scheduler.
     */
    inline bool getRunning()
    {
        return running_;
    }

    /*!
     * \brief Destructeur.
     */
    virtual ~ITimerScheduledListener()
    {
    }

protected:

    std::string name_;
    long period_us_;

    std::atomic<bool> paused_;
    std::atomic<bool> requestToStop_;
    std::atomic<bool> running_;

    /*!
     * \brief Prochain tick prevu (microsecondes depuis le chrono du scheduler).
     *        Gere par ActionManagerTimerThread.
     */
    uint64_t next_tick_us_;

    /*!
     * \brief Initialisation : nom et periode.
     */
    void init(std::string label, uint32_t period_us)
    {
        name_ = label;
        period_us_ = period_us;
        paused_ = false;
        requestToStop_ = false;
        running_ = false;
        next_tick_us_ = 0;
    }

    /*!
     * \brief Constructeur.
     */
    ITimerScheduledListener() :
            name_("ITimerScheduledListener_default"),
            period_us_(0),
            paused_(false),
            requestToStop_(false),
            running_(false),
            next_tick_us_(0)
    {
    }
};

#endif
