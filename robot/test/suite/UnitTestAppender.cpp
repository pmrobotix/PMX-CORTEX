/*!
 * \file
 * \brief Implémentation de la classe UnitTestAppender.
 */

#include <list>
#include <sstream>
#include <unistd.h>

#include "../../src/common/log/Level.hpp"
#include "UnitTestAppender.hpp"

// Codes ANSI pour la colorisation console
#define ANSI_RESET   "\033[0m"
#define ANSI_DIM     "\033[2m"        // gris (DEBUG)
#define ANSI_ORANGE  "\033[38;5;208m" // orange (WARN)
#define ANSI_RED     "\033[31m"       // rouge (ERROR)

UnitTestAppender::UnitTestAppender()
{
    this->indent_ = 0;
}

UnitTestAppender::~UnitTestAppender()
{
}

void UnitTestAppender::writeMessage(const logs::Logger &logger, const logs::Level &level, const std::string &message)
{
    // Colorisation si la sortie est un terminal
    const char *colorStart = "";
    const char *colorEnd = "";
    if (isatty(STDOUT_FILENO)) {
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
    }

    if (level == logs::Level::TELEM)
        return;

    std::string coloredMessage = std::string(colorStart) + message + colorEnd;
    logs::MemoryAppender::writeMessage(logger, level, coloredMessage);
}

void UnitTestAppender::increaseIndent()
{
    this->lock();
    this->indent_++;
    this->unlock();
}

void UnitTestAppender::decreaseIndent()
{
    this->lock();
    this->indent_--;
    if (this->indent_ < 0) {
        this->indent_ = 0;
    }
    this->unlock();
}

bool UnitTestAppender::expectedError(const std::string &message)
{
    this->lock();
    for (std::list<std::string>::iterator it = messages_.begin(); it != messages_.end(); it++) {
        if (*it == message) {
            this->messages_.erase(it);
            this->unlock();
            return true;
        }
    }
    this->unlock();
    return false;
}

void UnitTestAppender::cleanMessages()
{
    this->lock();
    this->messages_.clear();
    this->unlock();
}
