/*!
 * \file OPOS6UL_IAExtended.hpp
 * \brief Extension de l'intelligence artificielle pour la carte OPOS6UL.
 */

#ifndef OPOS6UL_IAEXTENDED_HPP_
#define OPOS6UL_IAEXTENDED_HPP_

#include <pmr_symmetrical_pg.h>
#include <string>

#include "ia/IAbyPath.hpp"
#include "ia/IAbyZone.hpp"

class Robot;

/*!
 * \brief Gestion de l'IA etendue du robot OPOS6UL (terrain de jeu, deplacement adverse).
 */
class OPOS6UL_IAExtended
{
private:

    IAbyZone iaz_; //old IA
    IAbyPath iap_; //new IA

    SymmetricalPlayground *p_;

public:

    PlaygroundObjectID opponent_1;
    PlaygroundObjectID opponent_2;
    PlaygroundObjectID opponent_3;
    PlaygroundObjectID opponent_4;

    /*!
     * \brief Zone dynamique de test (NON permanente), declaree avant
     * compute_edges puis desactivee. Restera inactive en production —
     * O_StrategyJsonRunnerTest l'active uniquement pour le scenario SR09
     * (PathImpossible / SK_IMPS). Permet de tester proprement le code
     * SK_IMPS du runner sans toucher aux bordures permanentes.
     */
    PlaygroundObjectID area_test_blocker;

    PlaygroundObjectID area_B4;
    PlaygroundObjectID area_C4;
    PlaygroundObjectID area_B3;

    PlaygroundObjectID area_A3;

    OPOS6UL_IAExtended(std::string botId, Robot *robot);

    ~OPOS6UL_IAExtended()
    {
    }

    /*!
     * \brief Retourne l'IA par zones (ancienne version).
     */
    IAbyZone& iAbyZone()
    {
        return iaz_;
    }

    /*!
     * \brief Retourne l'IA par pathfinding (nouvelle version).
     */
    IAbyPath& iAbyPath()
    {
        return iap_;
    }

    /*!
     * \brief Initialise le terrain de jeu (zones, obstacles, chemins).
     */
    void initPlayground();

    /*!
     * \brief Deplace la position connue de l'adversaire sur le terrain.
     * \param x_mm Position X de l'adversaire en mm.
     * \param y_mm Position Y de l'adversaire en mm.
     */
    void move_adv(float x_mm, float y_mm);
};

#endif
