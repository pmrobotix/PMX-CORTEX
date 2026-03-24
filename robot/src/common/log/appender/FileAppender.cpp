/*!
 * \file
 * \brief Implémentation de la classe FileAppender.
 */

#include "FileAppender.hpp"

#include <list>
#include <thread>

logs::FileAppender::~FileAppender()
{
    flush();
    lockMessages();
    ofs_.close();
    unlockMessages();
}

void logs::FileAppender::writeMessage(const logs::Logger & logger, const logs::Level & level,
        const std::string & message)
{
    logs::MemoryAppender::writeMessageOnly(message);
}

void logs::FileAppender::flush()
{
    lockMessages();
    while (this->messages_.size() > 0) {
        std::string message = this->messages_.front();
        ofs_ << message << std::endl;
        this->messages_.pop_front();
        std::this_thread::yield();
    }
    unlockMessages();
    ofs_.flush();
}
