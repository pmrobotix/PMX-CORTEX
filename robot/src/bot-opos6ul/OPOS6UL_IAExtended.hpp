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

    /*!
     * \brief Grenier : element fixe du jeu, centre sur l'axe X (x=1500),
     *        enabled + permanent au boot. Rectangle centre (1500, 295),
     *        1814 x 590. Le flag permanent empeche tout DELETE_ZONE JSON
     *        de l'eteindre au match.
     */
    PlaygroundObjectID area_zone_grenier;

    /*!
     * \brief Zones de prise P1..P4 : symetriques (id stocke = cote bleu).
     *        enabled au boot, desactivables via JSON ELEMENT/DELETE_ZONE
     *        item_id="zone_Pn" apres avoir pris. Le nom JSON reste "bleu"
     *        en match jaune : IAbyPath redirige automatiquement vers l'id
     *        miroir cote jaune.
     */
    PlaygroundObjectID area_zone_P1;  ///< centre bleu (175, 800),   450 x 500
    PlaygroundObjectID area_zone_P2;  ///< centre bleu (175, 1600),  450 x 500
    PlaygroundObjectID area_zone_P3;  ///< centre bleu (1100, 1825), 500 x 450
    PlaygroundObjectID area_zone_P4;  ///< centre bleu (1150, 1200), 500 x 450

    /*!
     * \brief Zones de depose GM2..GM6 (id stocke = cote bleu pour GM2/3/4,
     *        unique pour GM5/GM6 deja sur axe central x=1500).
     *        disabled au boot, activables via JSON ELEMENT/ADD_ZONE
     *        item_id="zone_GMn" apres avoir depose pour eviter que le
     *        pathfinding repasse dessus.
     */
    PlaygroundObjectID area_zone_GM2;  ///< sym, centre bleu (175, 1200), 450 x 500
    PlaygroundObjectID area_zone_GM3;  ///< sym, centre bleu (800, 1200), 500 x 450
    PlaygroundObjectID area_zone_GM4;  ///< sym, centre bleu (700, 1825), 500 x 450
    PlaygroundObjectID area_zone_GM5;  ///< unique, centre (1500, 1825), 500 x 450
    PlaygroundObjectID area_zone_GM6;  ///< unique, centre (1500, 1200), 500 x 450

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
