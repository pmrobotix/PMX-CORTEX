/*!
 * \file OPOS6UL_AsservExtended.hpp
 * \brief Extension de l'asservissement pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_ASSERVEXTENDED_HPP_
#define OPOS6UL_ASSERVEXTENDED_HPP_

#include <string>

#include "asserv/Asserv.hpp"
#include "log/LoggerFactory.hpp"

class OPOS6UL_RobotExtended;

/*!
 * \brief Gestion de l'asservissement étendu pour le robot OPOS6UL.
 *
 * Ajoute le filtrage de position sur la table et la mise à jour de la position adverse.
 */
class OPOS6UL_AsservExtended: public Asserv
{
private:
    /*!
     * \brief Return \ref Logger linked to \ref OPOS6UL_AsservExtended.
     */
    static inline const logs::Logger& logger()
    {
        static const logs::Logger &instance = logs::LoggerFactory::logger("OPOS6UL_AsservExtended");
        return instance;
    }

    OPOS6UL_RobotExtended *robot_extended_;

public:
    OPOS6UL_AsservExtended(std::string botId, OPOS6UL_RobotExtended *robot);

    ~OPOS6UL_AsservExtended()
    {
    }

    /*!
     * \brief Démarre le timer de mouvement et l'odométrie.
     * \param assistedHandlingEnabled Active le mode de conduite assistée (évitement).
     */
    void startMotionTimerAndOdo(bool assistedHandlingEnabled);

    /*!
     * \brief Met à jour la position de l'adversaire détecté.
     * \todo Déplacer cette méthode dans les actions.
     */
    void update_adv();
};

#endif
