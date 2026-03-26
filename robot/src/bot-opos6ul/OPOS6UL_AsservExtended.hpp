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
     * \brief Vérifie si des coordonnées (repère table) sont à l'intérieur du terrain.
     * \param x_botpos_repereTable Position X dans le repère table (mm).
     * \param y_botpos_repereTable Position Y dans le repère table (mm).
     * \return true si la position est dans les limites du terrain.
     */
    bool filtre_IsInsideTableXY(int x_botpos_repereTable, int y_botpos_repereTable);

    /*!
     * \brief Vérifie si une détection capteur correspond à une position dans le terrain.
     * \param dist_detect_mm Distance de détection en mm.
     * \param lateral_pos_sensor_mm Position latérale du capteur en mm.
     * \param desc Description optionnelle pour le log.
     * \return true si la position détectée est dans les limites du terrain.
     * \deprecated Utiliser filtre_IsInsideTableXY à la place.
     */
    bool filtre_IsInsideTable(int dist_detect_mm, int lateral_pos_sensor_mm, std::string desc = "");

    /*!
     * \brief Met à jour la position de l'adversaire détecté.
     * \todo Déplacer cette méthode dans les actions.
     */
    void update_adv();
};

#endif
