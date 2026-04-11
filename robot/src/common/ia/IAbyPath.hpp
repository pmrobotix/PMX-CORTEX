/*
 * IAbyPath.hpp
 *
 *  Created on: 4 mai 2017
 *      Author: pmx
 */

#ifndef COMMON_IA_IABYPATH_HPP_
#define COMMON_IA_IABYPATH_HPP_

#include <pmr_playground.h>
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

public:
    IAbyPath(Robot *robot);

    ~IAbyPath()
    {
    }

    void ia_start();
    void toSVG();

    void addPlayground(Playground * p);

    void ia_clear();
    void ia_createZone(const char* name, float minX, float minY, float width, float height, float startX, float startY,
            float startAngleDeg);
    ZONE* ia_getZone(const char* zoneName);
    ZONE* ia_getZoneAt(float x, float y);
    ZONE* ia_getNearestZoneFrom(float x, float y);

    void ia_addAction(const char* name, RobotAction action);

    void ia_printZone(ZONE *z);
    void ia_checkZones();

    void goToZone(const char *zoneName, ROBOTPOSITION *zone_p);

    void playgroundFindPath(FoundPath * & path, Point& start, Point& end);
    void enable(PlaygroundObjectID id, bool enable);


    // [DEPRECATED] Remplace par Navigator::pathTo() / pathBackTo() / pathToAndFaceTo() / pathToAndRotateAbsDeg()
    // Conserve en commentaire pour reference (bug detection adversaire, voir ARCHITECTURE.md).
    //
    // TRAJ_STATE doPathForwardAndFaceTo(float xMM, float yMM, float f_x, float f_y);
    // TRAJ_STATE doPathForwardAndRotateTo(float x, float y, float AbsThetaInDegree);
    // TRAJ_STATE doPathForwardTo(float xMM, float yMM, bool rotate_ignored_detection = false);
    // TRAJ_STATE doPathBackwardTo(float xMM, float yMM, bool rotate_ignored_detection = false);





//    TRAJ_STATE whileDoLine(float distMM, bool rotate_ignoring_opponent = true, int wait_tempo_us = 2000000,
//    		int nb_near_obstacle = 2, int nb_collision = 2, int reculOnObstacleMm = 0, int reculOnCollisionMm = 0,
//    		bool ignore_collision = 0);
    // [DEPRECATED] Supprime — utiliser Navigator avec RetryPolicy a la place
    // Voir Navigator::moveForwardTo(), goTo(), moveBackwardTo(), goBackTo(),
    //      rotateAbsDeg(), moveForwardToAndRotateAbsDeg(), etc.
    //
    // TRAJ_STATE whileMoveForwardTo(...);
    // TRAJ_STATE whileMoveBackwardTo(...);
    // TRAJ_STATE whileMoveRotateTo(...);
    // TRAJ_STATE whileMoveForwardAndRotateTo(...);
    // TRAJ_STATE whileMoveBackwardAndRotateTo(...);



};

#endif /* COMMON_IA_IABYPATH_HPP_ */
