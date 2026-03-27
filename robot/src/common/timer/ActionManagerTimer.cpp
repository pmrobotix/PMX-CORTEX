/*!
 * \file
 * \brief Implementation de la classe ActionManagerTimer.
 */

#include "ActionManagerTimer.hpp"
#include <semaphore.h>

ActionManagerTimer::ActionManagerTimer() :
        stopActionsAndTimers_(false), pause_(false), chronoTimer_("ActionManagerTimer")
{
    //init du semaphore valeur 0 pour une pause par defaut.
    sem_init(&AMT, 0, 0);
    //post increment
    //wait decremente
    //1 = running
    //0 = pause
}

void ActionManagerTimer::execute()
{
    int sizeT = 0;
    int sizePT = 0;
    int sizeA = 0;

    chronoTimer_.start();

    while (!stopActionsAndTimers_) {
        if (pause_) {
            sem_wait(&AMT);
        }

        if (!stopActionsAndTimers_ && (sizeA == 0) && (sizeT == 0) && (sizePT == 0)) {
            sem_wait(&AMT);
        } else {
            this->yield();
            std::this_thread::yield();
            utils::Thread::sleep_for_micros(1);
        }

        //on laisse le temps de faire autre chose si besoin
        this->yield();
        std::this_thread::yield();

        mtimer_.lock();
        sizePT = ptimers_tobestarted_.size();

        //on parcours la liste de timer POSIX a demarrer
        if (sizePT > 0 && !stopActionsAndTimers_) {
            while (sizePT != 0) {
                ITimerPosixListener *ptimer = ptimers_tobestarted_.front();
                ptimers_tobestarted_.pop_front();
                if (ptimer == NULL) {
                    logger().error("ptimers_tobestarted_ contains NULL, skipping");
                    sizePT = ptimers_tobestarted_.size();
                    continue;
                }
                if (!ptimer->getRunning()) {
                    ptimer->startTimer();
                } else {
                    logger().error() << "ptimers already started, name=" << ptimer->name() << logs::end;
                }
                ptimers_.push_back(ptimer);
                sizePT = ptimers_tobestarted_.size();

                std::this_thread::yield();
            }
        }
        mtimer_.unlock();

        //on traite les actions
        maction_.lock();
        // On teste s'il y a une tache a faire
        sizeA = actions_.size();
        if (sizeA > 0 && !stopActionsAndTimers_) {
            while (sizeA != 0) {
                IAction *action = actions_.front();
                if (action == NULL) {
                    logger().error("action is NULL, skipping");
                    actions_.pop_front();
                    sizeA = actions_.size();
                    continue;
                } else {
                    actions_.pop_front();
                    maction_.unlock();

                    bool persistaction = action->execute();

                    maction_.lock();
                    if (persistaction == true) {
                        actions_.push_back(action);
                        utils::Thread::sleep_for_micros(100);        //permet de ne pas avoir 100% du processeur
                        std::this_thread::yield();
                    }
                }
                sizeA = actions_.size(); //Maj

                std::this_thread::yield();
            }
        }
        maction_.unlock();
    }

    stopActionsAndTimers_ = false; //on reinitialise le stop.
}

void ActionManagerTimer::stopTimer(std::string timerNameToDelete)
{
    bool found = false;
    utils::PointerList<ITimerListener*>::iterator save;
    mtimer_.lock();
    utils::PointerList<ITimerListener*>::iterator i = timers_.begin();
    while (i != timers_.end()) {
        ITimerListener *timer = *i;
        if (timer->name() == timerNameToDelete) {
            save = i;
            found = true;
            timer->onTimerEnd(chronoTimer_);
            std::this_thread::yield();
        }
        i++;
    }
    if (found)
        timers_.erase(save);
    else
        logger().debug() << "Timer [" << timerNameToDelete << "] not found or already deleted." << logs::end;

    mtimer_.unlock();
}

void ActionManagerTimer::stopPTimer(std::string ptimerNameToDelete)
{

    bool found = false;
    utils::PointerList<ITimerPosixListener*>::iterator save;
    mtimer_.lock();
    utils::PointerList<ITimerPosixListener*>::iterator i = ptimers_.begin();
    while (i != ptimers_.end()) {
        ITimerPosixListener *ptimer = *i;
        if (ptimer->name() == ptimerNameToDelete) {
            save = i;
            found = true;
            ptimer->onTimerEnd(chronoTimer_);
            ptimer->remove();
        }
        i++;
        std::this_thread::yield();
    }
    if (found)
        ptimers_.erase(save);
    else
        logger().debug() << "PTimer [" << ptimerNameToDelete << "] not found or already deleted." << logs::end;

    mtimer_.unlock();
    unblock("stopPTimer");

}

bool ActionManagerTimer::findPTimer(std::string timerNameToFind)
{

    bool found = false;
    utils::PointerList<ITimerPosixListener*>::iterator save;
    mtimer_.lock();
    utils::PointerList<ITimerPosixListener*>::iterator i = ptimers_.begin();
    while (i != ptimers_.end()) {
        ITimerPosixListener *ptimer = *i;
        if (ptimer->name() == timerNameToFind) {
            save = i;
            found = true;
        }
        i++;
        std::this_thread::yield();
    }

    mtimer_.unlock();
    unblock("stopPTimer");
    return found;

}
void ActionManagerTimer::stopAllPTimers()
{
    mtimer_.lock();
    ptimers_tobestarted_.clear();
    while (ptimers_.size() != 0) {
        ITimerPosixListener *ptimer = ptimers_.front();
        ptimers_.pop_front();
        if (ptimer == NULL) {
            logger().error("ptimers_ contains NULL, skipping");
            continue;
        }
        //appel de la fonction de fin
        ptimer->onTimerEnd(chronoTimer_);
        //suppression du timer en cours d'execution
        ptimer->remove();
        std::this_thread::yield();
    }
    ptimers_.clear();

    while (timers_.size() != 0) {
        ITimerListener *timer = timers_.front();
        timers_.pop_front();
        if (timer == NULL) {
            logger().error("timers_ contains NULL, skipping");
            continue;
        }
        //appel de la fonction de fin
        timer->onTimerEnd(chronoTimer_);
    }
    timers_.clear();
    mtimer_.unlock();

    unblock("stopAllPTimers");

}

void ActionManagerTimer::stop()
{
    this->stopActionsAndTimers_ = true;
    stopAllPTimers();

    this->waitForEnd();
}

void ActionManagerTimer::pause(bool value)
{
    this->pause_ = value;
    //on parcours et on mets en pause tous les timers courants
    mtimer_.lock();
    utils::PointerList<ITimerPosixListener*>::iterator i = ptimers_.begin();
    while (i != ptimers_.end()) {
        ITimerPosixListener *ptimer = *i;
        ptimer->setPause(value);
        i++;
        std::this_thread::yield();
    }
    mtimer_.unlock();
    unblock("pause");
}

void ActionManagerTimer::debugActions()
{

    std::ostringstream temp;
    temp << "Print current actions";
    maction_.lock();
    utils::PointerList<IAction*>::iterator i = actions_.begin();
    while (i != actions_.end()) {
        IAction *action = *i;
        temp << " - " << action->info();
        i++;
        std::this_thread::yield();
    }
    maction_.unlock();
    logger().debug() << temp.str() << logs::end;
}

void ActionManagerTimer::debugPTimers()
{
    std::ostringstream temp;
    temp << "Print posixtimers to be started :";
    mtimer_.lock();
    if (ptimers_tobestarted_.size() != 0) {

        utils::PointerList<ITimerPosixListener*>::iterator i = ptimers_tobestarted_.begin();
        while (i != ptimers_tobestarted_.end()) {
            ITimerPosixListener *ptimer = *i;
            temp << " - " << ptimer->name();
            i++;
            std::this_thread::yield();
        }
    }
    mtimer_.unlock();
    logger().debug() << temp.str() << logs::end;

    std::ostringstream temp2;
    temp2 << "Print current posixtimers :";
    mtimer_.lock();
    if (ptimers_.size() != 0) {
        utils::PointerList<ITimerPosixListener*>::iterator ii = ptimers_.begin();
        while (ii != ptimers_.end()) {
            ITimerPosixListener *ptimer = *ii;
            temp2 << " - " << ptimer->name();
            ii++;
            std::this_thread::yield();
        }
    }
    mtimer_.unlock();
    logger().debug() << temp2.str() << logs::end;
}

//deprecated
void ActionManagerTimer::debugTimers()
{
    std::ostringstream temp;
    temp << "Print Defined timers :";
    mtimer_.lock();
    if (timers_.size() != 0) {
        utils::PointerList<ITimerListener*>::iterator i = timers_.begin();
        while (i != timers_.end()) {
            ITimerListener *timer = *i;
            temp << " - " << timer->name();
            i++;
        }
    }
    mtimer_.unlock();
    logger().debug() << temp.str() << logs::end;
}
