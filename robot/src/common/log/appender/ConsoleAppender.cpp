/*!
 * \file
 * \brief Implémentation de la classe ConsoleAppender.
 */

#include "ConsoleAppender.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <list>
#include <thread>

#include "../Level.hpp"

// Codes ANSI pour la colorisation console
#define ANSI_RESET   "\033[0m"
#define ANSI_DIM     "\033[2m"        // gris (DEBUG)
#define ANSI_ORANGE  "\033[38;5;208m" // orange (WARN)
#define ANSI_RED     "\033[31m"       // rouge (ERROR)

logs::ConsoleAppender::~ConsoleAppender() {
    flush();
}

void logs::ConsoleAppender::flush() {
    lockMessages();

    while (this->messages_.size() > 0) {
        std::string message = this->messages_.front();
        std::cout << message << std::endl;

        this->messages_.pop_front();
        std::this_thread::yield();
    }
    unlockMessages();
}

void logs::ConsoleAppender::writeMessage(const logs::Logger &logger, const logs::Level &level, const std::string &message) {
    if (level == logs::Level::TELEM)
        return;

    using namespace std::chrono;
    system_clock::time_point t = system_clock::now();
    long duration = (duration_cast<microseconds>(t - start_).count());

    // Colorisation de la ligne entiere selon le niveau
    const char *colorStart = "";
    const char *colorEnd = "";

    if (level == logs::Level::DEBUG) {
        colorStart = ANSI_DIM;
        colorEnd = ANSI_RESET;
    } else if (level == logs::Level::WARN) {
        colorStart = ANSI_ORANGE;
        colorEnd = ANSI_RESET;
    } else if (level == logs::Level::ERROR) {
        colorStart = ANSI_RED;
        colorEnd = ANSI_RESET;
    }

    std::ostringstream out;
    out << colorStart
        << std::setw(9) << duration << "| " << logger.name() << " " << level.name() << " " << message
        << colorEnd;

    this->lockMessages();
    this->messages_.push_back(out.str());
    this->unlockMessages();
}
