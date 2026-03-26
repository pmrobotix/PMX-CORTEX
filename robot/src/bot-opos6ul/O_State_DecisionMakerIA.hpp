/*!
 * \file
 * \brief Déclaration de la classe O_State_DecisionMakerIA.
 */

#ifndef O_STATE_DECISIONMAKERIA_HPP
#define	O_STATE_DECISIONMAKERIA_HPP

//TODO: migrer PathFinding include
//#include <src/pmr_playground.h>
#include <string>

#include "log/LoggerFactory.hpp"
#include "thread/Thread.hpp"

#define NO_ROTATION_DETECTION true
#define ROTATION_WITH_DETECTION false

#define WITH_PATHFINDING true
#define NO_PATHFINDING false

//#define COLLISION_IGNORE true
//#define COLLISION_WITH false


class Robot;

/*!
 * \brief Moteur de décision IA du robot.
 *
 * Ce thread gère la prise de décision stratégique en configurant
 * et exécutant les activités selon les zones de la table.
 */
class O_State_DecisionMakerIA: public utils::Thread
{
private:

    static inline const logs::Logger & logger()
    {
        static const logs::Logger & instance = logs::LoggerFactory::logger("O_State_DecisionMakerIA");
        return instance;
    }

    /*!
     * \brief Référence vers le robot.
     */
    Robot & robot_;


public:

    O_State_DecisionMakerIA(Robot& robot);

    ~O_State_DecisionMakerIA()
    {
    }

    /*!
     * \brief Execute l'action.
     */
    void execute();

    std::string name()
    {
        return "O_State_DecisionMakerIA";
    }

    /*!
     * \brief Configure les activités pour les zones de jeu du match.
     */
    void IASetupActivitiesZone();

    /*!
     * \brief Configure les activités pour le mode test de la table.
     */
    void IASetupActivitiesZoneTableTest();

};

#endif
