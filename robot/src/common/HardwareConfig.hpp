/*!
 * \file
 * \brief Configuration dynamique des drivers hardware.
 *
 * Lit un fichier hardware.conf a cote de l'executable pour determiner
 * quels drivers sont actifs (ARM reel) ou desactives (stub simu).
 * Si le fichier est absent, tous les drivers sont actifs par defaut.
 */

#ifndef HARDWARECONFIG_HPP_
#define HARDWARECONFIG_HPP_

#include <map>
#include <string>

#include "log/LoggerFactory.hpp"

/*!
 * \brief Singleton de configuration hardware.
 *
 * Charge hardware.conf au demarrage et expose isEnabled()
 * pour chaque driver. Les factory create() consultent cette
 * classe pour decider d'instancier le driver reel ou le stub.
 */
class HardwareConfig
{
public:

    /*!
     * \brief Retourne l'instance unique.
     */
    static HardwareConfig& instance();

    /*!
     * \brief Charge la configuration depuis le repertoire de l'executable.
     * \param exePath argv[0] ou chemin vers l'executable.
     */
    void load(const std::string& exePath);

    /*!
     * \brief Teste si un driver est active.
     * \param driverName Nom du driver (ex: "LedDriver", "LcdShieldDriver").
     * \return true si active ou si la config n'a pas ete chargee (defaut = tout ON).
     */
    bool isEnabled(const std::string& driverName) const;

private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("HardwareConfig");
        return instance;
    }

    HardwareConfig() = default;

    std::map<std::string, bool> drivers_;
    bool loaded_ = false;
};

#endif
