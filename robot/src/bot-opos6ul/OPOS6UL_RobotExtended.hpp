/*!
 * \file OPOS6UL_RobotExtended.hpp
 * \brief Extension de la classe Robot pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_ROBOTEXTENDED_HPP_
#define OPOS6UL_ROBOTEXTENDED_HPP_

//#include "interface/AAsservDriver.hpp"
#include "Robot.hpp"
#include "log/LoggerFactory.hpp"

class O_State_DecisionMakerIA;
class OPOS6UL_ActionsExtended;
class OPOS6UL_AsservExtended;
class OPOS6UL_IAExtended;
class OPOS6UL_SvgWriterExtended;

/*!
 * \brief Classe principale du robot étendu pour OPOS6UL (singleton).
 *
 * Gère l'initialisation, les actions, l'IA et l'affichage du robot.
 */
class OPOS6UL_RobotExtended: public Robot
{
public:

    /*!
     * \brief Retourne l'instance unique du robot (singleton).
     * \return Référence vers l'instance unique.
     */
    static OPOS6UL_RobotExtended& instance()
    {
        static OPOS6UL_RobotExtended instance;
        return instance;
    }

    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("OPOS6UL_RobotExtended");
        return instance;
    }

    ~OPOS6UL_RobotExtended();

    inline OPOS6UL_ActionsExtended& actions()
    {
        OPOS6UL_ActionsExtended &r_actions = *p_actions_;
        return r_actions;
    }

//    inline OPOS6UL_ActionsExtended& actions()
//    {
//        OPOS6UL_ActionsExtended& r_actions = *p_actions_;
//        return r_actions;
//    }
//
//    inline OPOS6UL_AsservExtended& asserv()
//    {
//        OPOS6UL_AsservExtended& r_asserv = *p_asserv_;
//        return r_asserv;
//    }

    inline OPOS6UL_IAExtended& ia()
    {
        OPOS6UL_IAExtended &r_ia = *p_ia_;
        return r_ia;
    }

    /*!
     * \brief Initialise l'état du robot et le décisionmaker IA.
     * \param argc Nombre d'arguments de la ligne de commande.
     * \param argv Tableau des arguments de la ligne de commande.
     */
    void begin(int argc, char **argv);

    /*!
     * \brief Affiche les points accumulés pendant le match.
     */
    void displayPoints();

    /*!
     * \brief Arrête toutes les actions supplémentaires (capteurs, LEDs, servos).
     */
    void stopExtraActions();

    /*!
     * \brief Réinitialise l'affichage du statut de trajectoire.
     */
    void resetDisplayTS();

    /*!
     * \brief Affiche le statut de trajectoire courant.
     * \param ts État de la trajectoire à afficher.
     */
    void displayTS(TRAJ_STATE ts);

    /*!
     * \brief Réinitialise l'affichage de détection d'obstacle.
     */
    void resetDisplayObstacle();

    /*!
     * \brief Affiche le niveau de détection d'obstacle.
     * \param level Niveau de l'obstacle détecté.
     */
    void displayObstacle(int level);

    O_State_DecisionMakerIA *decisionMaker_; ///< Pointeur vers le décisionmaker IA.

    bool force_end_of_match; ///< Force la fin du match (arrêt d'urgence).

private:

    OPOS6UL_ActionsExtended *p_actions_;

    //OPOS6UL_AsservExtended *p_asserv_;

    OPOS6UL_IAExtended *p_ia_;

    //OPOS6UL_SvgWriterExtended *p_svg_;

    OPOS6UL_RobotExtended();
};

#endif
