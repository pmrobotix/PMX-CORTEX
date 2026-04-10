/*!
 * \file
 * \brief Définition de la classe Actions.
 */

#ifndef ACTIONS_HPP_
#define ACTIONS_HPP_

#include <string>

#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"
#include "timer/ActionTimerScheduler.hpp"
#include "LedBar.hpp"

class IAction;

/*!
 * List of robot actions. It contains all common RobotElement.
 */
class Actions
{
private:

    /*!
     * \brief Return \ref Logger linked to \ref Actions.
     */
    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("Actions");
        return instance;
    }

    /*!
     * \brief Scheduler unique : 1 thread pour toutes les IAction et tous
     *        les ITimerScheduledListener.
     */
    ActionTimerScheduler actionScheduler_;

public:

    /*!
     * \brief Constructor.
     *
     */
    Actions();

    /*!
     * \brief Destructor.
     */
    ~Actions()
    {
    }

    /*!
     * \brief Ajout d'une action.
     * \param action L'action à ajouter.
     */
    inline void addAction(IAction *action)
    {
        if (!this->is_started()) {
            logger().error() << "addAction ACTIONMANAGER NOT STARTED !!" << logs::end;
        }
        actionScheduler_.addAction(action);
    }

    /*!
     * \brief Ajout d'un timer dans le scheduler unique.
     * \param timer Le ITimerScheduledListener a ajouter.
     */
    inline void addTimer(ITimerScheduledListener *timer)
    {
        actionScheduler_.addTimer(timer);
    }

    /*!
     * \brief Acces direct au scheduler (pour stopTimer, findTimer, debug).
     */
    inline ActionTimerScheduler& scheduler()
    {
        return actionScheduler_;
    }

    /*!
     * \brief Active les actions.
     *
     * Cette méthode lance le thread gérant le ActionManager.
     */
    void start();

    /*!
     * \brief Indique si le gestionnaire d'actions est démarré.
     * \return \c true si démarré, \c false sinon.
     */
    bool is_started();

    /*!
     * \brief supprime toutes les actions et les timers.
     */
    void clearAll();
    /*!
     * \brief Arrete le thread sensorManager et actionManager.
     */
    void waitAndStopManagers();

    /*!
     * \brief Annule le thread en cours.
     */
    void cancel();

};

#endif
