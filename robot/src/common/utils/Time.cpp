/*!
 * \file
 * \brief Implementation des fonctions de pause clock_nanosleep.
 */

#include "Time.hpp"
#include <ctime>

void utils::nanosleep_micros(int64_t usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
}

void utils::nanosleep_millis(int64_t msec) {
    struct timespec ts;
    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
}
