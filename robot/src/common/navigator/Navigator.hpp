#ifndef COMMON_NAVIGATOR_NAVIGATOR_HPP_
#define COMMON_NAVIGATOR_NAVIGATOR_HPP_

#include <functional>
#include <vector>

#include "log/LoggerFactory.hpp"
#include "interface/AAsservDriver.hpp"
#include "navigator/RetryPolicy.hpp"

class Robot;
class IAbyPath;

/*!
 * \brief Point de passage pour manualPath() / pathTo().
 */
struct Waypoint
{
    float x;
    float y;
    bool reverse = false;
};

/*!
 * \brief Mode d'execution des waypoints.
 *
 * - STOP : envoi un par un, arret a chaque point
 * - CHAIN : envoi groupe, arret a chaque point
 * - CHAIN_NONSTOP : envoi groupe, trajectoire fluide (le plus rapide)
 */
enum PathMode
{
    STOP,            //!< Envoi un par un, arret a chaque point
    CHAIN,           //!< Envoi groupe, arret a chaque point
    CHAIN_NONSTOP    //!< Envoi groupe, pas d'arret (le plus rapide)
};

/*!
 * \brief Navigation unifiee avec retry.
 *
 * Factorise la logique de retry (while + obstacle + collision + recul)
 * en une seule methode executeWithRetry(), utilisee par tous les types
 * de deplacement.
 *
 * Remplace :
 *   - Robot::whileDoLine()
 *   - IAbyPath::whileMoveForwardTo / BackwardTo / RotateTo / ForwardAndRotateTo / BackwardAndRotateTo
 *
 * Dependances :
 *   - Asserv (via Robot*) pour les mouvements directs
 *   - IAbyPath pour le calcul de chemin A*
 */
class Navigator
{
private:

    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("Navigator");
        return instance;
    }

    Robot* robot_;
    IAbyPath* iap_;

    /*!
     * \brief Boucle de retry unique pour tous les types de deplacement.
     */
    TRAJ_STATE executeWithRetry(std::function<TRAJ_STATE()> moveFunc,
                                const RetryPolicy& policy,
                                int reculDir = -1);

    /*!
     * \brief Calcule les waypoints via IAbyPath::playgroundFindPath().
     */
    std::vector<Waypoint> computePath(float x, float y, bool reverse = false);

    /*!
     * \brief Execute les waypoints selon le PathMode (sans retry, appele par executeWithRetry).
     */
    TRAJ_STATE executeWaypoints(const std::vector<Waypoint>& waypoints,
                                const RetryPolicy& policy,
                                PathMode mode,
                                size_t& currentIndex,
                                int svgBotColor = 0);

    /*!
     * \brief Trace les segments waypoints sur le SVG.
     */
    void svgTraceWaypoints(float x_start, float y_start,
                           const std::vector<Waypoint>& waypoints,
                           const std::string& color, float width, bool dashed);

public:

    Navigator(Robot* robot, IAbyPath* iap = NULL);
    ~Navigator();

    // ========== MOUVEMENTS SIMPLES (defaut: pas de retry) ==========

    /*!
     * \brief Ligne droite.
     * \param distMm Distance en mm (positif=avancer, negatif=reculer).
     */
    TRAJ_STATE line(float distMm, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Avance vers (x,y) : faceTo + gotoXY.
     */
    TRAJ_STATE goTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Recule vers (x,y) : reverseFaceTo + gotoReverse.
     */
    TRAJ_STATE goToReverse(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    // ========== ROTATIONS (defaut: pas de retry) ==========

    /*!
     * \brief Rotation relative en degres.
     */
    TRAJ_STATE rotateDeg(float degRelative, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Rotation absolue en degres (repere terrain).
     */
    TRAJ_STATE rotateToAbsoluteDeg(float thetaDeg, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Tourne face a un point (x,y).
     */
    TRAJ_STATE faceTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Tourne dos a un point (x,y).
     */
    TRAJ_STATE reverseFaceTo(float x, float y, RetryPolicy policy = RetryPolicy::noRetry());

    /*!
     * \brief Rotation orbitale asservie autour d'une roue (pivot asservi).
     * \param angleDeg Angle en degres.
     * \param forward true = marche avant.
     * \param turnRight true = pivot roue droite.
     */
    TRAJ_STATE orbitalTurnDeg(float angleDeg, bool forward, bool turnRight, RetryPolicy policy = RetryPolicy::noRetry());

    // ========== SUITE DE WAYPOINTS MANUELS ==========

    /*!
     * \brief Execute une liste de waypoints.
     * \param waypoints Liste de points.
     * \param policy    Parametres de retry.
     * \param mode      STOP / NONSTOP / CHAIN / CHAIN_NONSTOP.
     */
    TRAJ_STATE manualPath(const std::vector<Waypoint>& waypoints,
                          RetryPolicy policy = RetryPolicy::standard(),
                          PathMode mode = STOP);

    // ========== PATHFINDING A* (via IAbyPath) ==========

    /*!
     * \brief Calcule un chemin A* puis execute les waypoints.
     * Le chemin est recalcule a chaque tentative de retry.
     */
    TRAJ_STATE pathTo(float x, float y,
                      RetryPolicy policy = RetryPolicy::standard(),
                      PathMode mode = STOP);

    /*!
     * \brief Idem pathTo en marche arriere.
     */
    TRAJ_STATE pathToReverse(float x, float y,
                             RetryPolicy policy = RetryPolicy::standard(),
                             PathMode mode = STOP);

    // ========== COMBINAISONS MOUVEMENT + ROTATION FINALE ==========

    TRAJ_STATE goToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE goToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE goToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());

    TRAJ_STATE pathToAndRotateAbsDeg(float x, float y, float thetaDeg, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE pathToAndRotateRelDeg(float x, float y, float degRelative, RetryPolicy policy = RetryPolicy::standard());
    TRAJ_STATE pathToAndFaceTo(float x, float y, float fx, float fy, RetryPolicy policy = RetryPolicy::standard());
};

#endif
