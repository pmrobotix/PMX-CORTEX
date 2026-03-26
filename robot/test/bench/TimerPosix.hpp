/*!
 * \file
 * \brief Timer POSIX standalone pour bench.
 *
 * Exemple de timer POSIX brut (sans ActionManagerTimer).
 * Utilise pour verifier le comportement bas niveau des signaux POSIX.
 */

#ifndef BENCH_TIMERPOSIX_HPP_
#define BENCH_TIMERPOSIX_HPP_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include "utils/Chronometer.hpp"

class TimerPosix {
public:
    TimerPosix(std::string id) :
             id_(id), memberVariable(0)
    {
        this->timerSpecs.it_value.tv_sec = 1;
        this->timerSpecs.it_value.tv_nsec = 0;
        this->timerSpecs.it_interval.tv_sec = 2;
        this->timerSpecs.it_interval.tv_nsec = 0;

        sigemptyset(&this->SignalAction.sa_mask);
        this->SignalAction.sa_flags = SA_SIGINFO;
        this->SignalAction.sa_sigaction = TimerPosix::alarmFunction;

        memset(&this->signalEvent, 0, sizeof(this->signalEvent));
        this->signalEvent.sigev_notify = SIGEV_SIGNAL;
        this->signalEvent.sigev_value.sival_ptr = (void*) this;
        this->signalEvent.sigev_signo = SIGALRM;

        if (timer_create(CLOCK_REALTIME, &this->signalEvent, &this->timerID) != 0) {
            perror("Could not create the timer");
            sleep(1);
            exit(1);
        }

        if (sigaction(SIGALRM, &this->SignalAction, NULL)) {
            perror("Could not install new signal handler");
        }
    }

    void start() {
        if (timer_settime(this->timerID, 0, &this->timerSpecs, NULL) == -1) {
            perror("Could not start timer:");
        }
        if (!chrono.started()) chrono.start();
    }

    static void alarmFunction(int sigNumb, siginfo_t *si, void *uc) {
        TimerPosix *ptrTimerPosix = reinterpret_cast<TimerPosix*>(si->si_value.sival_ptr);
        ptrTimerPosix->memberAlarmFunction();
    }

    timer_t timerID;
    sigset_t SigBlockSet;
    struct sigevent signalEvent;
    struct sigaction SignalAction;
    struct itimerspec timerSpecs;

    static utils::Chronometer chrono;
    std::string id_;

private:
    int memberVariable;

    void memberAlarmFunction() {
        int c = this->chrono.getElapsedTimeInMicroSec();
        this->memberVariable++;
        std::cout << id_ << " " << c << " n=" << this->memberVariable << std::endl;
    }
};

#endif
