/*!
 * \file Navigator.cpp
 * \brief Implementation de la classe Navigator.
 */

#include "navigator/Navigator.hpp"

#include <cmath>
#include <thread>

#include <pmr_node.h>
#include <pmr_path_result.h>
#include <pmr_point.h>

#include "log/Logger.hpp"
#include "log/SvgWriter.hpp"
#include "asserv/Asserv.hpp"
#include "ia/IAbyPath.hpp"
#include "Robot.hpp"
#include "utils/Chronometer.hpp"
#include "thread/Thread.hpp"

Navigator::Navigator(Robot* robot, IAbyPath* iap)
    : robot_(robot), iap_(iap)
{
}

Navigator::~Navigator()
{
}

// =============================================================================
// executeWithRetry — coeur unique de la logique de retry
// =============================================================================

TRAJ_STATE Navigator::executeWithRetry(std::function<TRAJ_STATE()> moveFunc,
                                        const RetryPolicy& policy,
                                        int reculDir)
{
    TRAJ_STATE ts = TRAJ_IDLE;
    int obstacleCount = 0;
    int collisionCount = 0;

    while (ts != TRAJ_FINISHED)
    {
        ts = moveFunc();

        robot_->svgPrintPosition();
        robot_->displayTS(ts);

        if (ts == TRAJ_FINISHED)
            break;

        if (ts == TRAJ_IMPOSSIBLE)
        {
            logger().info() << "TRAJ_IMPOSSIBLE" << logs::end;
            robot_->asserv().resetEmergencyOnTraj("Navigator TRAJ_IMPOSSIBLE");
            break;
        }

        if (ts == TRAJ_NEAR_OBSTACLE)
        {
            obstacleCount++;
            logger().info() << "OBSTACLE essai " << obstacleCount
                            << "/" << policy.maxObstacleRetries << logs::end;

            utils::sleep_for_micros(policy.waitTempoUs);

            if (obstacleCount < policy.maxObstacleRetries)
            {
                robot_->asserv().resetEmergencyOnTraj("Navigator OBSTACLE retry");
            }

            if (policy.reculObstacleMm > 0)
            {
                float reculDist = reculDir * policy.reculObstacleMm;
                TRAJ_STATE tr = robot_->asserv().line(reculDist);
                if (tr != TRAJ_FINISHED && tr != TRAJ_IDLE)
                {
                    robot_->asserv().resetEmergencyOnTraj("Navigator recul obstacle");
                }
            }

            if (obstacleCount >= policy.maxObstacleRetries)
                break;
        }

        if (ts == TRAJ_COLLISION)
        {
            if (policy.ignoreCollision)
            {
                robot_->asserv().resetEmergencyOnTraj("Navigator COLLISION ignored");
                continue;
            }

            collisionCount++;
            logger().info() << "COLLISION essai " << collisionCount
                            << "/" << policy.maxCollisionRetries << logs::end;

            utils::sleep_for_micros(policy.waitTempoUs);

            if (collisionCount < policy.maxCollisionRetries)
            {
                robot_->asserv().resetEmergencyOnTraj("Navigator COLLISION retry");
            }

            if (policy.reculCollisionMm > 0)
            {
                float reculDist = reculDir * policy.reculCollisionMm;
                TRAJ_STATE tr = robot_->asserv().line(reculDist);
                if (tr != TRAJ_FINISHED && tr != TRAJ_IDLE)
                {
                    robot_->asserv().resetEmergencyOnTraj("Navigator recul collision");
                }
            }

            if (collisionCount >= policy.maxCollisionRetries)
                break;
        }

        robot_->resetDisplayTS();
        std::this_thread::yield();
    }

    robot_->displayTS(ts);
    logger().debug() << "time=" << robot_->chrono().getElapsedTimeInMilliSec() << "ms"
                     << " x=" << robot_->asserv().pos_getX_mm()
                     << " y=" << robot_->asserv().pos_getY_mm()
                     << " a=" << robot_->asserv().pos_getThetaInDegree() << logs::end;

    return ts;
}

// =============================================================================
// executeWaypoints — execute les waypoints selon le PathMode
// =============================================================================

TRAJ_STATE Navigator::executeWaypoints(const std::vector<Waypoint>& waypoints,
                                        const RetryPolicy& policy,
                                        PathMode mode,
                                        size_t& currentIndex,
                                        int svgBotColor)
{
    bool isChain = (mode == CHAIN || mode == CHAIN_NONSTOP);
    bool isNonstop = (mode == CHAIN_NONSTOP);

    // ---- Mode CHAIN / CHAIN_NONSTOP : envoi groupe puis waitTraj ----
    if (isChain)
    {
        // FaceTo vers le premier waypoint avant d'envoyer la queue
        if (currentIndex < waypoints.size())
        {
            const Waypoint& first = waypoints[currentIndex];
            float x_match = robot_->asserv().changeMatchX(first.x);
            if (policy.rotateIgnoringOpponent)
            {
                TRAJ_STATE ts = robot_->asserv().faceTo(x_match, first.y, first.reverse);
                if (ts != TRAJ_FINISHED)
                {
                    return ts;
                }
                robot_->svgPrintPosition();
            }
        }

        // Envoi de tous les waypoints sans attendre
        for (size_t i = currentIndex; i < waypoints.size(); i++)
        {
            const Waypoint& wp = waypoints[i];
            bool isLast = (i == waypoints.size() - 1);
            float x_match = robot_->asserv().changeMatchX(wp.x);

            if (wp.reverse)
            {
                if (isNonstop && !isLast)
                    robot_->asserv().goToReverseChainSend(x_match, wp.y);
                else
                    robot_->asserv().goToReverseSend(x_match, wp.y);
            }
            else
            {
                if (isNonstop && !isLast)
                    robot_->asserv().goToChainSend(x_match, wp.y);
                else
                    robot_->asserv().goToSend(x_match, wp.y);
            }
        }

        // Attendre la fin de la queue
        TRAJ_STATE ts = robot_->asserv().waitTraj();
        robot_->svgPrintPosition(svgBotColor);

        if (ts != TRAJ_FINISHED)
        {
            currentIndex = waypoints.size() - 1;
        }

        return ts;
    }

    // ---- Mode STOP : envoi un par un, bloquant ----
    for (size_t i = currentIndex; i < waypoints.size(); i++)
    {
        const Waypoint& wp = waypoints[i];
        TRAJ_STATE ts;

        float x_match = robot_->asserv().changeMatchX(wp.x);

        // Rotation vers le point
        if (policy.rotateIgnoringOpponent)
        {
            ts = robot_->asserv().faceTo(x_match, wp.y, wp.reverse);
            if (ts != TRAJ_FINISHED)
            {
                currentIndex = i;
                return ts;
            }
            robot_->svgPrintPosition();
        }

        // Deplacement
        if (wp.reverse)
            ts = robot_->asserv().goToReverse(x_match, wp.y);
        else
            ts = robot_->asserv().goTo(x_match, wp.y);

        robot_->svgPrintPosition(svgBotColor);

        if (ts != TRAJ_FINISHED)
        {
            currentIndex = i;
            return ts;
        }
    }

    return TRAJ_FINISHED;
}

// =============================================================================
// svgTraceWaypoints — trace les segments sur le SVG
// =============================================================================

void Navigator::svgTraceWaypoints(float x_start, float y_start,
                                   const std::vector<Waypoint>& waypoints,
                                   const std::string& color, float width, bool dashed)
{
    float x_prev = x_start;
    float y_prev = y_start;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        float x_wp = robot_->asserv().changeMatchX(waypoints[i].x);
        float y_wp = waypoints[i].y;
        robot_->svgw().writeLine(x_prev, y_prev, x_wp, y_wp, color, width, dashed);
        x_prev = x_wp;
        y_prev = y_wp;
    }
}

// =============================================================================
// Mouvements simples
// =============================================================================

TRAJ_STATE Navigator::line(float distMm, RetryPolicy policy)
{
    float x_init = robot_->asserv().pos_getX_mm();
    float y_init = robot_->asserv().pos_getY_mm();
    float d_restant = distMm;
    int reculDir = (distMm >= 0) ? -1 : 1;

    return executeWithRetry(
        [this, distMm, x_init, y_init, &d_restant]() {
            TRAJ_STATE ts = robot_->asserv().line(d_restant);
            // Recalcul distance restante
            float dx = robot_->asserv().pos_getX_mm() - x_init;
            float dy = robot_->asserv().pos_getY_mm() - y_init;
            float parcourue = std::sqrt(dx * dx + dy * dy);
            d_restant = distMm - parcourue;
            logger().debug() << "d_parcourue=" << parcourue
                             << " d_restant=" << d_restant << logs::end;
            return ts;
        },
        policy,
        reculDir
    );
}

TRAJ_STATE Navigator::goTo(float x, float y, RetryPolicy policy)
{
    float x_before = robot_->asserv().pos_getX_mm();
    float y_before = robot_->asserv().pos_getY_mm();

    TRAJ_STATE ts = executeWithRetry(
        [this, x, y, &policy]() {
            return robot_->asserv().moveForwardTo(x, y, policy.rotateIgnoringOpponent);
        },
        policy,
        -1
    );

    robot_->svgw().writeLine(x_before, y_before,
        robot_->asserv().pos_getX_mm(), robot_->asserv().pos_getY_mm(),
        "blue", 2);
    robot_->svgPrintPosition(4); // BLUE

    return ts;
}

TRAJ_STATE Navigator::goToReverse(float x, float y, RetryPolicy policy)
{
    float x_before = robot_->asserv().pos_getX_mm();
    float y_before = robot_->asserv().pos_getY_mm();

    TRAJ_STATE ts = executeWithRetry(
        [this, x, y, &policy]() {
            return robot_->asserv().moveBackwardTo(x, y, policy.rotateIgnoringOpponent);
        },
        policy,
        1
    );

    robot_->svgw().writeLine(x_before, y_before,
        robot_->asserv().pos_getX_mm(), robot_->asserv().pos_getY_mm(),
        "blue", 2);
    robot_->svgPrintPosition(4); // BLUE

    return ts;
}

// =============================================================================
// Rotations
// =============================================================================

TRAJ_STATE Navigator::rotateDeg(float degRelative, RetryPolicy policy)
{
    return executeWithRetry(
        [this, degRelative, &policy]() {
            return robot_->asserv().rotateDeg(degRelative, policy.rotateIgnoringOpponent);
        },
        policy,
        0
    );
}

TRAJ_STATE Navigator::rotateAbsDeg(float thetaDeg, RetryPolicy policy)
{
    return executeWithRetry(
        [this, thetaDeg, &policy]() {
            return robot_->asserv().rotateAbsDeg(thetaDeg, policy.rotateIgnoringOpponent);
        },
        policy,
        0
    );
}

TRAJ_STATE Navigator::faceTo(float x, float y, RetryPolicy policy)
{
    return executeWithRetry(
        [this, x, y]() {
            return robot_->asserv().faceTo(x, y, false);
        },
        policy,
        0
    );
}

TRAJ_STATE Navigator::reverseFaceTo(float x, float y, RetryPolicy policy)
{
    return executeWithRetry(
        [this, x, y]() {
            return robot_->asserv().faceTo(x, y, true);
        },
        policy,
        0
    );
}

TRAJ_STATE Navigator::orbitalTurnDeg(float angleDeg, bool forward, bool turnRight, RetryPolicy policy)
{
    return executeWithRetry(
        [this, angleDeg, forward, turnRight]() {
            return robot_->asserv().orbitalTurnDeg(angleDeg, forward, turnRight);
        },
        policy,
        0
    );
}

// =============================================================================
// manualPath — execute une liste de waypoints
// =============================================================================

TRAJ_STATE Navigator::manualPath(const std::vector<Waypoint>& waypoints, RetryPolicy policy, PathMode mode)
{
    if (waypoints.empty())
        return TRAJ_FINISHED;

    // Tracer sur le SVG : CHAIN = vert continu, CHAIN_NONSTOP = vert pointille, STOP = pas de trace
    if (mode == CHAIN || mode == CHAIN_NONSTOP)
    {
        svgTraceWaypoints(robot_->asserv().pos_getX_mm(), robot_->asserv().pos_getY_mm(),
                          waypoints, "green", 3, mode == CHAIN_NONSTOP);
    }

    size_t currentIndex = 0;

    return executeWithRetry(
        [this, &waypoints, &currentIndex, &policy, mode]() {
            return executeWaypoints(waypoints, policy, mode, currentIndex, 3); // GREEN
        },
        policy,
        -1
    );
}

// =============================================================================
// Pathfinding A*
// =============================================================================

std::vector<Waypoint> Navigator::computePath(float x, float y, bool reverse)
{
    std::vector<Waypoint> waypoints;

    Point startPoint = {
        robot_->asserv().pos_getX_mm(),
        robot_->asserv().pos_getY_mm()
    };
    Point endPoint = {
        robot_->asserv().changeMatchX(x),
        y
    };

    FoundPath* found_path = NULL;
    iap_->playgroundFindPath(found_path, startPoint, endPoint);

    if (found_path != NULL)
    {
        if (found_path->cost > 0)
        {
            std::vector<Node*>::iterator it;
            int count = 0;
            for (it = found_path->path.begin(); it < found_path->path.end(); it++)
            {
                Node* node = *it;
                if (count != 0)
                {
                    Waypoint wp;
                    wp.x = robot_->asserv().changeMatchX(node->x);
                    wp.y = node->y;
                    wp.reverse = reverse;
                    waypoints.push_back(wp);
                }
                count++;
            }
        }
        delete found_path;
    }

    return waypoints;
}

TRAJ_STATE Navigator::pathTo(float x, float y, RetryPolicy policy, PathMode mode)
{
    return executeWithRetry(
        [this, x, y, &policy, mode]() {
            std::vector<Waypoint> waypoints = computePath(x, y, false);
            if (waypoints.empty())
                return TRAJ_IMPOSSIBLE;

            // Tracer le chemin A* sur le SVG (rouge, pointille si CHAIN_NONSTOP)
            svgTraceWaypoints(robot_->asserv().pos_getX_mm(), robot_->asserv().pos_getY_mm(),
                              waypoints, "red", 4, mode == CHAIN_NONSTOP);

            size_t idx = 0;
            return executeWaypoints(waypoints, policy, mode, idx, 2); // RED
        },
        policy,
        -1
    );
}

TRAJ_STATE Navigator::pathToReverse(float x, float y, RetryPolicy policy, PathMode mode)
{
    return executeWithRetry(
        [this, x, y, &policy, mode]() {
            std::vector<Waypoint> waypoints = computePath(x, y, true);
            if (waypoints.empty())
                return TRAJ_IMPOSSIBLE;

            // Tracer le chemin A* sur le SVG (rouge, pointille si CHAIN_NONSTOP)
            svgTraceWaypoints(robot_->asserv().pos_getX_mm(), robot_->asserv().pos_getY_mm(),
                              waypoints, "red", 4, mode == CHAIN_NONSTOP);

            size_t idx = 0;
            return executeWaypoints(waypoints, policy, mode, idx, 2); // RED
        },
        policy,
        1
    );
}

// =============================================================================
// Combinaisons mouvement + rotation finale
// =============================================================================

TRAJ_STATE Navigator::goToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy)
{
    TRAJ_STATE ts = goTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return rotateAbsDeg(thetaDeg, policy);
}

TRAJ_STATE Navigator::goToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy)
{
    TRAJ_STATE ts = goTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return rotateDeg(degRelative, policy);
}

TRAJ_STATE Navigator::goToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy)
{
    TRAJ_STATE ts = goTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return faceTo(fx, fy, policy);
}

TRAJ_STATE Navigator::pathToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy)
{
    TRAJ_STATE ts = pathTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return rotateAbsDeg(thetaDeg, policy);
}

TRAJ_STATE Navigator::pathToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy)
{
    TRAJ_STATE ts = pathTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return rotateDeg(degRelative, policy);
}

TRAJ_STATE Navigator::pathToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy)
{
    TRAJ_STATE ts = pathTo(x, y, policy);
    if (ts != TRAJ_FINISHED)
        return ts;
    return faceTo(fx, fy, policy);
}
