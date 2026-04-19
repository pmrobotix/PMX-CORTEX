#include "ZoneJsonExporter.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

#include <pmr_playground.h>
#include <pmr_pathfinding.h>
#include <pmr_zone.h>
#include <pmr_node.h>

#include "log/LoggerFactory.hpp"
#include "IAbyPath.hpp"
#include "IACommon.hpp"

static inline const logs::Logger& logger()
{
    static const logs::Logger& instance = logs::LoggerFactory::logger("ZoneJsonExporter");
    return instance;
}

static void writePolygonFromZone(std::ostringstream& out, Zone* z)
{
    out << "\"points\": [";
    for (unsigned int i = 0; i < z->nodes_count; ++i) {
        if (i > 0) out << ",";
        out << "{\"x\":" << z->nodes[i]->x << ",\"y\":" << z->nodes[i]->y << "}";
    }
    out << "]";
}

static void writeRectZone(std::ostringstream& out, const ZONE* z)
{
    float x0 = z->minX,          y0 = z->minY;
    float x1 = z->minX + z->width, y1 = z->minY + z->height;
    out << "\"points\": ["
        << "{\"x\":" << x0 << ",\"y\":" << y0 << "},"
        << "{\"x\":" << x1 << ",\"y\":" << y0 << "},"
        << "{\"x\":" << x1 << ",\"y\":" << y1 << "},"
        << "{\"x\":" << x0 << ",\"y\":" << y1 << "}"
        << "]";
}

bool ZoneJsonExporter::exportToFile(const std::string& path, Playground* pg, IAbyPath* ia)
{
    if (pg == nullptr || ia == nullptr) {
        std::cerr << "[ZoneJsonExporter] playground or ia is null" << std::endl;
        logger().error() << "exportToFile: playground or ia is null" << logs::end;
        return false;
    }

    std::ostringstream out;
    out << "{\n";
    out << "  \"sizeX\": 3000,\n";
    out << "  \"sizeY\": 2000,\n";
    out << "  \"color0\": \"bleu\",\n";
    out << "  \"color3000\": \"jaune\",\n";
    out << "  \"marge\": 0,\n";

    // forbiddenZones : obstacles du pathfinding
    out << "  \"forbiddenZones\": [\n";
    PathFinder* pf = pg->get_path_finder();
    if (pf != nullptr) {
        int count = 0;
        for (Zone* z : pf->zones) {
            if (z == nullptr || z->nodes_count == 0) continue;
            if (count > 0) out << ",\n";
            out << "    {"
                << "\"id\":\"pg_zone_" << z->id << "\","
                << "\"forme\":\"polygone\","
                << "\"desc\":\"Playground obstacle " << z->id << "\","
                << "\"type\":\"all\","
                << "\"active\":" << (z->enabled ? "true" : "false") << ","
                ;
            writePolygonFromZone(out, z);
            out << "}";
            ++count;
        }
        out << "\n";
    }
    out << "  ],\n";

    // dynamicZones : zones d'action IA
    out << "  \"dynamicZones\": [\n";
    int nzones = ia->zonesCount();
    ZONE* const* zones = ia->zones();
    for (int i = 0; i < nzones; ++i) {
        const ZONE* z = zones[i];
        if (z == nullptr) continue;
        if (i > 0) out << ",\n";
        out << "    {"
            << "\"id\":\"" << z->name << "\","
            << "\"forme\":\"polygone\","
            << "\"desc\":\"IA zone " << z->name << "\","
            << "\"active\":true,"
            << "\"startX\":" << z->startX << ","
            << "\"startY\":" << z->startY << ","
            << "\"startAngle\":" << z->startAngle << ","
            ;
        writeRectZone(out, z);
        out << "}";
    }
    out << "\n  ],\n";

    out << "  \"detectionIgnoreZone\": []\n";
    out << "}\n";

    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ZoneJsonExporter] cannot open " << path << std::endl;
        logger().error() << "exportToFile: cannot open " << path << logs::end;
        return false;
    }
    f << out.str();
    f.close();

    const size_t nForbidden = (pf ? pf->zones.size() : 0);
    std::cerr << "[ZoneJsonExporter] wrote " << path
              << " forbidden=" << nForbidden << " dynamic=" << nzones << std::endl;
    logger().info() << "Zones exported to " << path
                    << " (forbidden=" << nForbidden
                    << ", dynamic=" << nzones << ")"
                    << logs::end;
    return true;
}
