/*!
 * \file
 * \brief Implémentation de la classe Chronometer.
 */

#include "Chronometer.hpp"

#include <chrono>
#include <thread>
#include <iostream>

utils::Chronometer::Chronometer(std::string name) :
        stopped_(1), timerPeriod_us_(0), name_(name), periodNb_(0), endSetTime_us(0), timerStartTime_us_(0), lastTime_(0)
{
    startCount_.tv_sec = 0;
    startCount_.tv_usec = 0;
    endCount_.tv_sec = 0;
    endCount_.tv_usec = 0;
    endSet_.tv_sec = 0;
    endSet_.tv_usec = 0;
}

void utils::Chronometer::start()
{
    stopped_ = 0;
    gettimeofday(&startCount_, NULL);
    lastTime_ = getElapsedTimeInMicroSec();
}

void utils::Chronometer::stop()
{
    stopped_ = 1;
    gettimeofday(&endCount_, NULL);
}

unsigned long long utils::Chronometer::getElapsedTimeInMicroSec()
{
    if (!stopped_) {
        gettimeofday(&endCount_, NULL);
    }
    long seconds = endCount_.tv_sec - startCount_.tv_sec;
    return seconds * 1000000 + (endCount_.tv_usec - startCount_.tv_usec);
}

float utils::Chronometer::getElapsedTimeInMilliSec()
{
    return this->getElapsedTimeInMicroSec() * 0.001;
}

float utils::Chronometer::getElapsedTimeInSec()
{
    return this->getElapsedTimeInMicroSec() * 0.000001;
}

float utils::Chronometer::getElapsedTime()
{
    return this->getElapsedTimeInSec();
}

timeval utils::Chronometer::getTime()
{
    timeval result;
    gettimeofday(&result, NULL);
    return result;
}

int utils::Chronometer::waitTimer(int delay_us, bool debug)
{
    periodNb_++;
    unsigned long long endTaskTime = getElapsedTimeInMicroSec();
    int t = (timerPeriod_us_ - (int) (endTaskTime - lastTime_)) - delay_us;

    if (t > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(t));
    } else {
        if (t > 2)
            std::cerr << "OVERFLOW " << name_ << " " << t / 1000 << std::endl;
    }

    lastTime_ = getElapsedTimeInMicroSec();
    return periodNb_;
}

void utils::Chronometer::setTimer(unsigned int usec)
{
    timerPeriod_us_ = usec;
    this->start();
    timerStartTime_us_ = 0;
}
