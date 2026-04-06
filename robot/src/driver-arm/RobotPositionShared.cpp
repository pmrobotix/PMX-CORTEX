/*!
 * \file
 * \brief Implementation ARM de la position partagee du robot.
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
    pushHistory(p, (uint32_t)(t_set_us_ / 1000));
    this->unlock();
}

ROBOTPOSITION RobotPositionShared::getRobotPosition(int debug)
{
    ROBOTPOSITION p;
    unsigned long long t_get_us;
    this->lock();
    p = p_;
    t_get_us = chrono_.getElapsedTimeInMicroSec();
    this->unlock();
    if (debug == 1)
        printf("--getRobotPosition  p.x=%f p.y=%f diffT=%ld \n", p.x, p.y, (long) (t_get_us - t_set_us_));

    return p;
}
