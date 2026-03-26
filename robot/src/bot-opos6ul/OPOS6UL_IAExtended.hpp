/*!
 * \file OPOS6UL_IAExtended.hpp
 * \brief Extension de l'intelligence artificielle pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_IAEXTENDED_HPP_
#define OPOS6UL_IAEXTENDED_HPP_

//TODO: migrer le module IA (IAbyPath, IAbyZone, PathFinding)

#include <string>

class Robot;

/*!
 * \brief Gestion de l'IA étendue du robot OPOS6UL (terrain de jeu, déplacement adverse).
 *
 * \todo Migrer les modules IAbyPath, IAbyZone et PathFinding.
 */
class OPOS6UL_IAExtended
{
public:

    OPOS6UL_IAExtended(std::string botId, Robot *robot);

    ~OPOS6UL_IAExtended()
    {
    }

    /*!
     * \brief Initialise le terrain de jeu (zones, obstacles, chemins).
     */
    void initPlayground();

    /*!
     * \brief Déplace la position connue de l'adversaire sur le terrain.
     * \param x_mm Position X de l'adversaire en mm.
     * \param y_mm Position Y de l'adversaire en mm.
     */
    void move_adv(float x_mm, float y_mm);
};

#endif
