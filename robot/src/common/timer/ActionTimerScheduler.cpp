/*!
 * \file
 * \brief Implementation de ActionTimerScheduler.
 */

#include "ActionTimerScheduler.hpp"

#include <sstream>
#include <time.h>
#include <climits>

ActionTimerScheduler::ActionTimerScheduler() :
        stop_(false), pause_(false), chronoTimer_("ActionTimerScheduler")
{
    sem_init(&sem_, 0, 0);
}

void ActionTimerScheduler::execute()
{
    chronoTimer_.start();

    while (!stop_)
    {
        // 1. Pause : on attend qu'on nous reveille
        if (pause_) {
            sem_wait(&sem_);
            if (stop_) break;
        }

        // 2. Si rien a faire (ni action ni timer), on dort sur le semaphore
        maction_.lock();
        bool has_action = !actions_.empty();
        maction_.unlock();

        mtimer_.lock();
        bool has_timer = !timers_.empty() || !timers_to_add_.empty();
        mtimer_.unlock();

        if (!has_action && !has_timer) {
            sem_wait(&sem_);
            if (stop_) break;
            continue;
        }

        // 3. Transferer les nouveaux timers vers la liste active
        //    (init de next_tick_us_ pour le 1er tick immediat)
        mtimer_.lock();
        uint64_t now_us = chronoTimer_.getElapsedTimeInMicroSec();
        while (!timers_to_add_.empty()) {
            ITimerScheduledListener *t = timers_to_add_.front();
            timers_to_add_.pop_front();
            if (t == nullptr) continue;
            t->next_tick_us_ = now_us; // 1er tick immediat
            t->running_ = true;
            timers_.push_back(t);
        }
        mtimer_.unlock();

        // 4. Calculer le prochain tick le plus proche
        uint64_t next_deadline_us = now_us + 100000; // max 100ms si rien
        mtimer_.lock();
        for (auto i = timers_.begin(); i != timers_.end(); ++i) {
            ITimerScheduledListener *t = *i;
            if (t == nullptr || t->paused_) continue;
            if (t->next_tick_us_ < next_deadline_us)
                next_deadline_us = t->next_tick_us_;
        }
        mtimer_.unlock();

        // 5. Si des actions en attente : pas de sleep, on enchaine.
        //    Sinon : sleep jusqu'au prochain tick.
        if (!has_action && next_deadline_us > now_us) {
            // clock_nanosleep ABSTIME (CLOCK_MONOTONIC) — pas de drift
            // Conversion : on se base sur CLOCK_MONOTONIC absolu
            struct timespec target_abs;
            clock_gettime(CLOCK_MONOTONIC, &target_abs);
            uint64_t delta_us = next_deadline_us - now_us;
            target_abs.tv_sec  += delta_us / 1000000;
            target_abs.tv_nsec += (delta_us % 1000000) * 1000;
            if (target_abs.tv_nsec >= 1000000000) {
                target_abs.tv_sec++;
                target_abs.tv_nsec -= 1000000000;
            }
            // EINTR : on poursuit (le semaphore peut nous reveiller via signal)
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target_abs, nullptr);
        }

        if (stop_) break;

        // 6. Tick les timers prets + retirer ceux qui demandent l'arret
        now_us = chronoTimer_.getElapsedTimeInMicroSec();
        mtimer_.lock();
        utils::PointerList<ITimerScheduledListener*> to_remove;
        for (auto i = timers_.begin(); i != timers_.end(); ++i) {
            ITimerScheduledListener *t = *i;
            if (t == nullptr) {
                to_remove.push_back(t);
                continue;
            }
            if (t->requestToStop()) {
                t->onTimerEnd(chronoTimer_);
                t->running_ = false;
                to_remove.push_back(t);
                continue;
            }
            if (t->paused_) continue;
            if (t->next_tick_us_ <= now_us) {
                // Unlock pendant onTimer pour permettre add/remove pendant l'appel
                mtimer_.unlock();
                t->onTimer(chronoTimer_);
                mtimer_.lock();

                // Replanifie le prochain tick
                t->next_tick_us_ += t->period_us_;
                // Anti rattrapage : si on est tres en retard, on reprogramme depuis maintenant
                uint64_t now2_us = chronoTimer_.getElapsedTimeInMicroSec();
                if (t->next_tick_us_ < now2_us) {
                    t->next_tick_us_ = now2_us + t->period_us_;
                }
            }
        }
        // Suppression differee
        for (auto i = to_remove.begin(); i != to_remove.end(); ++i) {
            timers_.remove(*i);
        }
        mtimer_.unlock();

        if (stop_) break;

        // 7. Executer UNE seule action par cycle (pour ne pas bloquer les timers)
        maction_.lock();
        if (!actions_.empty()) {
            IAction *action = actions_.front();
            actions_.pop_front();
            if (action == nullptr) {
                maction_.unlock();
            } else {
                maction_.unlock();
                bool persist = action->execute();
                if (persist) {
                    maction_.lock();
                    actions_.push_back(action);
                    maction_.unlock();
                }
            }
        } else {
            maction_.unlock();
        }
    }

    // Sortie : appeler onTimerEnd sur tous les timers restants
    mtimer_.lock();
    while (!timers_.empty()) {
        ITimerScheduledListener *t = timers_.front();
        timers_.pop_front();
        if (t != nullptr) {
            t->onTimerEnd(chronoTimer_);
            t->running_ = false;
        }
    }
    timers_to_add_.clear();
    mtimer_.unlock();

    stop_ = false;
}

void ActionTimerScheduler::stopTimer(std::string timerName)
{
    bool found = false;
    mtimer_.lock();
    for (auto i = timers_.begin(); i != timers_.end(); ++i) {
        ITimerScheduledListener *t = *i;
        if (t != nullptr && t->name() == timerName) {
            t->requestStop(); // sera traite dans le prochain cycle
            found = true;
        }
    }
    for (auto i = timers_to_add_.begin(); i != timers_to_add_.end(); ++i) {
        ITimerScheduledListener *t = *i;
        if (t != nullptr && t->name() == timerName) {
            t->requestStop();
            found = true;
        }
    }
    mtimer_.unlock();
    if (!found) {
        logger().debug() << "stopTimer: [" << timerName << "] not found" << logs::end;
    }
    unblock("stopTimer");
}

bool ActionTimerScheduler::findTimer(std::string timerName)
{
    bool found = false;
    mtimer_.lock();
    for (auto i = timers_.begin(); i != timers_.end(); ++i) {
        if (*i != nullptr && (*i)->name() == timerName) { found = true; break; }
    }
    if (!found) {
        for (auto i = timers_to_add_.begin(); i != timers_to_add_.end(); ++i) {
            if (*i != nullptr && (*i)->name() == timerName) { found = true; break; }
        }
    }
    mtimer_.unlock();
    return found;
}

void ActionTimerScheduler::stopAllTimers()
{
    mtimer_.lock();
    for (auto i = timers_.begin(); i != timers_.end(); ++i) {
        if (*i != nullptr) (*i)->requestStop();
    }
    for (auto i = timers_to_add_.begin(); i != timers_to_add_.end(); ++i) {
        if (*i != nullptr) (*i)->requestStop();
    }
    mtimer_.unlock();
    unblock("stopAllTimers");
}

void ActionTimerScheduler::stop()
{
    stopAllTimers();
    stop_ = true;
    unblock("stop");
    this->waitForEnd();
}

void ActionTimerScheduler::debugActions()
{
    std::ostringstream oss;
    oss << "Actions:";
    maction_.lock();
    for (auto i = actions_.begin(); i != actions_.end(); ++i) {
        if (*i != nullptr) oss << " " << (*i)->info();
    }
    maction_.unlock();
    logger().debug() << oss.str() << logs::end;
}

void ActionTimerScheduler::debugTimers()
{
    std::ostringstream oss;
    oss << "Timers:";
    mtimer_.lock();
    for (auto i = timers_.begin(); i != timers_.end(); ++i) {
        if (*i != nullptr) oss << " " << (*i)->name() << "(" << (*i)->timeSpan_us() << "us)";
    }
    if (!timers_to_add_.empty()) {
        oss << " | to_add:";
        for (auto i = timers_to_add_.begin(); i != timers_to_add_.end(); ++i) {
            if (*i != nullptr) oss << " " << (*i)->name();
        }
    }
    mtimer_.unlock();
    logger().debug() << oss.str() << logs::end;
}
