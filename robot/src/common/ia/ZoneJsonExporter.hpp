#ifndef COMMON_IA_ZONE_JSON_EXPORTER_HPP_
#define COMMON_IA_ZONE_JSON_EXPORTER_HPP_

#include <string>

class Playground;
class IAbyPath;

/*!
 * Ecrit un table.json au format du simulateur PMX-CORTEX :
 *   - forbiddenZones : obstacles du Playground (bordures, adversaire, ...)
 *   - dynamicZones   : zones d'action declarees via IAbyPath::ia_createZone()
 *
 * Convention de coordonnees : PMX (mm, origine bas-gauche, table 3000 x 2000).
 * Le simulateur ajoute une margin visuelle (champ "marge") ; on met 0 ici car
 * les obstacles du Playground incluent deja la marge d'evitement.
 */
class ZoneJsonExporter
{
public:
    static bool exportToFile(const std::string& path, Playground* pg, IAbyPath* ia);
};

#endif
