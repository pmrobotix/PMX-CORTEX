/*!
 * \file
 * \brief Fonctions de pause basees sur clock_nanosleep (CLOCK_MONOTONIC).
 *
 * \warning Ces fonctions sont interruptibles par les signaux POSIX (EINTR),
 *          notamment les timers POSIX (SensorsThread). Preferer utils::sleep_for_micros()
 *          (dans Thread.hpp) sauf besoin explicite de clock_nanosleep
 *          (ex: boucle temps-reel avec PeriodicTimer).
 */

#ifndef UTILS_TIME_HPP
#define UTILS_TIME_HPP

#include <cstdint>

namespace utils {

/*!
 * \brief Met en pause le thread courant pendant \a usec microsecondes (clock_nanosleep).
 * \warning Interruptible par les signaux POSIX (EINTR).
 */
void nanosleep_micros(int64_t usec);

/*!
 * \brief Met en pause le thread courant pendant \a msec millisecondes (clock_nanosleep).
 * \warning Interruptible par les signaux POSIX (EINTR).
 */
void nanosleep_millis(int64_t msec);

}

#endif
