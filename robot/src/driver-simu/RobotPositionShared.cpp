/*!
 * \file
 * \brief Implementation SIMU de la position partagee du robot.
 */

#include "RobotPositionShared.hpp"

#include <cstdio>

ARobotPositionShared* ARobotPositionShared::create()
{
    static RobotPositionShared *instance = new RobotPositionShared();
    return instance;
}

RobotPositionShared::RobotPositionShared()
{
    t_set_us_ = 0;
    chrono_.start();
}

RobotPositionShared::~RobotPositionShared()
{
}

void RobotPositionShared::setRobotPosition(ROBOTPOSITION p)
{
    this->lock();
    p_ = p;
    t_set_us_ = chrono_.getElapsedTimeInMicroSec();
    this->unlock();
}

ROBOTPOSITION RobotPositionShared::getRobotPosition(int debug)
{
    ROBOTPOSITION p;
    this->lock();
    p = p_;
    this->unlock();

    return p;
}
