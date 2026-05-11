/*
 * IAbyPath.hpp
 *
 *  Created on: 4 mai 2017
 *      Author: pmx
 */

#ifndef COMMON_IA_IABYPATH_HPP_
#define COMMON_IA_IABYPATH_HPP_

#include <pmr_playground.h>
#include <map>
#include <sstream>
#include <string>

#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"
#include "../Robot.hpp"
#include "IACommon.hpp"


struct FoundPath;

struct Point;

struct RobotPosition;

class Playground;

class Robot;

class IAbyPath
{
private:
    /*!
     * \brief Retourne le \ref Logger associé à la classe \ref IAbyPath.
     */
    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("IAbyPath");
        return instance;
    }

    inline const logs::Logger & loggerSvg()
    {
        std::ostringstream s;
        s << "IAbyPath4" << robot_->getID();
        const logs::Logger & svg_ = logs::LoggerFactory::logger(s.str());
        return svg_;
    }

    Robot * robot_;
    Playground * p_;

    int _zones_count;
    ZONE* _zones[100];

    int _actions_count;
    ACTIONS* _actions[200];

    // Mapping nom string -> PlaygroundObjectID, alimente au boot par
    // OPOS6UL_IAExtended::initPlayground() pour permettre au runner JSON
    // (ELEMENT/ADD_ZONE, ELEMENT/DELETE_ZONE) d'activer/desactiver un
    // obstacle pathfinding par son nom.
    std::map<std::string, PlaygroundObjectID> namedObstacles_;

    // Mapping nom string -> (id cote bleu, id cote jaune) pour les zones
    // creees via add_rectangle_symmetrical. Le JSON utilise le nom "bleu"
    // (ex: "zone_P1"), enableNamedObstacle choisit le bon id en fonction
    // de la couleur de match courante (Robot::isMatchColor()).
    std::map<std::string, std::pair<PlaygroundObjectID, PlaygroundObjectID>> symObstacles_;

public:
    IAbyPath(Robot *robot);

    ~IAbyPath()
    {
    }

    void ia_start();

    /*!
     * \brief Genere svgIA.svg (ou \a path) reflechissant l'etat courant des
     *        zones du pathfinding. Distinction visuelle :
     *          - zone enabled  -> fill Aqua (obstacle actif)
     *          - zone disabled -> fill Silver (definie mais inactive)
     *        Ecriture atomique : la string SVG est construite puis ecrite
     *        dans "<path>.tmp" avant d'etre renommee vers \a path.
     */
    void toSVG(const std::string& path = "svgIA.svg");

    void addPlayground(Playground * p);

    /*!
     * \brief Retourne true si (x,y) tombe dans une zone marquee permanente
     *        (bordures, grenier, depart adverse). Utilise par
     *        StrategyJsonRunner::validateAgainstPlayground() pour detecter,
     *        au chargement de la strategie JSON, une cible inatteignable
     *        avant que le match commence. Retourne false si p_ == NULL ou
     *        si (x,y) hors de toute zone permanente.
     */
    bool pointInPermanentZone(float x, float y);

    void ia_clear();
    void ia_createZone(const char* name, float minX, float minY, float width, float height, float startX, float startY,
            float startAngleDeg);
    ZONE* ia_getZone(const char* zoneName);
    ZONE* ia_getZoneAt(float x, float y);
    ZONE* ia_getNearestZoneFrom(float x, float y);

    // Accesseurs pour export (cf. ZoneJsonExporter)
    int zonesCount() const { return _zones_count; }
    ZONE* const* zones() const { return _zones; }
    Playground* playground() const { return p_; }

    void ia_addAction(const char* name, RobotAction action);

    void ia_printZone(ZONE *z);
    void ia_checkZones();

    void goToZone(const char *zoneName, ROBOTPOSITION *zone_p);

    void playgroundFindPath(FoundPath * & path, Point& start, Point& end);
    void enable(PlaygroundObjectID id, bool enable);

    /*!
     * \brief Enregistre un obstacle pathfinding sous un nom string, afin
     *        de pouvoir l'activer/desactiver depuis une strategie JSON via
     *        ELEMENT/ADD_ZONE et ELEMENT/DELETE_ZONE.
     */
    void registerNamedObstacle(const std::string& name, PlaygroundObjectID id);

    /*!
     * \brief Enregistre une paire d'obstacles symetriques (cote bleu, cote
     *        jaune) sous un seul nom. La strategie JSON utilise toujours le
     *        nom "bleu" ; enableNamedObstacle redirige automatiquement vers
     *        l'id jaune si Robot::isMatchColor() retourne true (= JAUNE).
     */
    void registerSymmetricalObstacle(const std::string& name,
                                     PlaygroundObjectID idBlue,
                                     PlaygroundObjectID idYellow);

    /*!
     * \brief Active ou desactive un obstacle pathfinding enregistre par nom.
     *        Cherche d'abord dans les paires symetriques (et choisit l'id
     *        selon la couleur match), puis dans les obstacles simples.
     * \return true si le nom est connu, false sinon (et aucun effet).
     */
    bool enableNamedObstacle(const std::string& name, bool enabled);

    // Accesseurs lecture seule sur les maps de noms (utilises par
    // ZoneJsonExporter pour afficher les noms reels dans table.json
    // plutot que des ids techniques auto-generes).
    const std::map<std::string, PlaygroundObjectID>& namedObstacles() const
    {
        return namedObstacles_;
    }
    const std::map<std::string, std::pair<PlaygroundObjectID, PlaygroundObjectID>>&
    symObstacles() const
    {
        return symObstacles_;
    }



};

#endif /* COMMON_IA_IABYPATH_HPP_ */
