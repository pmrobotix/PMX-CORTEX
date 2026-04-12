/*!
 * \file
 * \brief Implémentation de la classe TelemetryAppender.
 */

#include "TelemetryAppender.hpp"

#include <list>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <iostream>

#include "../../utils/json.hpp"

using namespace std::chrono;

logs::TelemetryAppender::TelemetryAppender(std::string Id_Robot, std::string target_ip, int port)
{
    id_ = Id_Robot;

    t_fd = socket(AF_INET, SOCK_DGRAM, 0); //UDP
    strcpy(ip_, target_ip.c_str());

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);
    addr_.sin_addr.s_addr = inet_addr(ip_);
}

void logs::TelemetryAppender::configure(const std::string &target_ip, int port)
{
    strcpy(ip_, target_ip.c_str());
    addr_.sin_port = htons(port);
    addr_.sin_addr.s_addr = inet_addr(ip_);
}

void logs::TelemetryAppender::flush() {

    lockMessages();

    while (this->messagesjson_.size() > 0) {
        std::string message = this->messagesjson_.front();

        message += '\n';
        sendto(t_fd, message.c_str(), message.size(), 0, (struct sockaddr*) &addr_, sizeof(addr_));
        this->messagesjson_.pop_front();
        std::this_thread::yield();
    }

    while (this->messages_.size() > 0) {
        std::string message = this->messages_.front();
        std::cout << message << std::endl; //AFFICHAGE CONSOLE
        this->messages_.pop_front();
        std::this_thread::yield();
    }

    unlockMessages();

}

/*!
 * \brief Méthode générique de trace d'un message.
 * \param logger
 *        Logger de référence du message.
 * \param level
 *        Niveau de référence du message.
 * \param message
 *        Message à tracer.
 */
void logs::TelemetryAppender::writeMessage(const logs::Logger &logger, const logs::Level &level, const std::string &message) {

    if (level == logs::Level::TELEM or level == logs::Level::ERROR) {
        //LOG Telemetry
        writeMessageWithJsonTime(id_, logger, level, message);
    }

    if (level != logs::Level::TELEM) {
        //Log Console
        logs::MemoryAppender::writeMessage(logger, level, message);
    }
}

void logs::TelemetryAppender::writeMessageWithJsonTime(std::string id, const logs::Logger & logger, const logs::Level &level,
        const std::string & message)
{
    uint64_t duration = (duration_cast<microseconds>(system_clock::now() - start_).count());
    uint64_t ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    nlohmann::json j;
    j[id]["t"] = (double)(ms/1000.0); //timestamp systeme en secondes (3 decimales)
    j[id]["dt"] = (double)(duration/1000.0); //delta time depuis demarrage en ms

    if (level == logs::Level::TELEM) {
        try
        {
            j[id][logger.name()] = nlohmann::json::parse(message);
        }
        catch(const std::exception& e)
        {
            // Message TELEM non-JSON : stocker le message brut en fallback
            j[id][logger.name()] = message;
        }
    }
    else if (level == logs::Level::ERROR) {
        j[id][logger.name()]["ERROR"][message] = duration;
    }

    this->lockMessages();
    this->messagesjson_.push_back(j.dump());
    this->unlockMessages();

}
