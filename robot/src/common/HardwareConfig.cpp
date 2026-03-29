/*!
 * \file
 * \brief Implementation de HardwareConfig.
 */

#include "HardwareConfig.hpp"

#include <fstream>
#include <algorithm>

#include "log/Logger.hpp"

HardwareConfig& HardwareConfig::instance()
{
    static HardwareConfig inst;
    return inst;
}

void HardwareConfig::load(const std::string& exePath)
{
    // Determiner le repertoire de l'executable
    std::string dir;
    auto pos = exePath.rfind('/');
    if (pos != std::string::npos) {
        dir = exePath.substr(0, pos + 1);
    } else {
        dir = "./";
    }

    std::string filePath = dir + "hardware.conf";
    std::ifstream file(filePath);

    if (!file.is_open()) {
        logger().info() << "hardware.conf not found at " << filePath
                        << " => all drivers enabled (default)" << logs::end;
        loaded_ = false;
        return;
    }

    std::string summary;
    std::string line;
    while (std::getline(file, line)) {
        // Supprimer les espaces
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        // Ignorer commentaires et lignes vides
        if (line.empty() || line[0] == '#') {
            continue;
        }

        auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        std::string name = line.substr(0, eq);
        std::string value = line.substr(eq + 1);

        bool enabled = (value == "1");
        drivers_[name] = enabled;

        // Nom court : supprimer le suffixe "Driver"
        std::string shortName = name;
        auto drvPos = shortName.find("Driver");
        if (drvPos != std::string::npos) shortName.erase(drvPos);

        if (!summary.empty()) summary += " ";
        summary += shortName + "=" + (enabled ? "ON" : "OFF");
    }

    loaded_ = true;
    logger().info() << summary << logs::end;
}

bool HardwareConfig::isEnabled(const std::string& driverName) const
{
    // Si pas charge, tout est active (comportement par defaut = match)
    if (!loaded_) {
        return true;
    }

    auto it = drivers_.find(driverName);
    if (it == drivers_.end()) {
        // Driver absent du fichier = active par defaut
        return true;
    }

    return it->second;
}
