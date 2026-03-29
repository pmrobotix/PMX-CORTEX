/*!
 * \file
 * \brief Définition de la classe TelemetryAppender.
 */

#ifndef LOGS_TELEMETRYAPPENDER_HPP_
#define LOGS_TELEMETRYAPPENDER_HPP_


#include "../LoggerFactory.hpp"
#include "MemoryAppender.hpp"

#include <iostream>
#include <list>
#include <string>
#include <chrono>

#include <netinet/in.h>

#include "../../utils/json.hpp"

using namespace std::chrono;

namespace logs {
/*!
 * \brief Implémentation de Appender pour une écriture des traces
 * sur un flux de reseau.
 *
 */
class TelemetryAppender: public MemoryAppender {
private:

    /*!
     * \brief ID du ROBOT.
     */
    std::string id_;

    /*!
     * \brief Liste des messages json enregistrés.
     */
    std::list<std::string> messagesjson_;

    /*!
     * \brief IP reseau du récepteur de télémétrie (RPI).
     */
    char ip_[100];
    /*!
     * \brief Address socket UDP.
     */
    struct sockaddr_in addr_;
    /*!
     * \brief socket UDP.
     */
    int t_fd;

public:

    /*!
     * \brief Constructeur.
     * \param Id_Robot Identifiant du robot (clé racine du JSON).
     * \param target_ip Adresse IP du récepteur de télémétrie (ex: "192.168.3.101").
     */
    TelemetryAppender(std::string Id_Robot, std::string target_ip);

    /*!
     * \brief Destructeur de la classe.
     */
    virtual ~TelemetryAppender() {
        flush();
    }

    /*!
     * \brief Trace un message. Les messages TELEM et ERROR sont envoyés en UDP,
     *        les autres sont affichés sur la console.
     * \param logger Logger de référence du message.
     * \param level Niveau de référence du message.
     * \param message Message à tracer.
     */
    void writeMessage(const logs::Logger &logger, const logs::Level &level, const std::string &message);

    /*!
     * \brief Enregistre un message au format JSON avec horodatage pour la télémétrie.
     * \param id Identifiant du robot.
     * \param logger Logger de référence.
     * \param level Niveau de référence.
     * \param message Message à tracer (JSON pour TELEM, texte pour ERROR).
     */
    void writeMessageWithJsonTime(std::string id, const logs::Logger & logger, const logs::Level &level, const std::string & message);

    /*!
     * \brief Envoie tous les messages JSON en attente via UDP et affiche les messages console.
     */
    void flush();

};
}

#endif
