/*!
 * \file
 * \brief Implémentation de la classe OPOS6UL_IAExtended.
 */

#include "OPOS6UL_IAExtended.hpp"

#include <pmr_symmetrical_pg.h>

class Robot;

OPOS6UL_IAExtended::OPOS6UL_IAExtended(std::string botId, Robot *robot) :
        iaz_(robot), iap_(robot)
{
    opponent_1 = Playground::INVALID;
//    opponent_2 = Playground::INVALID;
//    opponent_3 = Playground::INVALID;
//    opponent_4 = Playground::INVALID;
    area_test_blocker = Playground::INVALID;
    area_zone_grenier = Playground::INVALID;
    area_zone_P1 = Playground::INVALID;
    area_zone_P2 = Playground::INVALID;
    area_zone_P3 = Playground::INVALID;
    area_zone_P4 = Playground::INVALID;
    area_zone_GM2 = Playground::INVALID;
    area_zone_GM3 = Playground::INVALID;
    area_zone_GM4 = Playground::INVALID;
    area_zone_GM5 = Playground::INVALID;
    area_zone_GM6 = Playground::INVALID;

    area_B4 = Playground::INVALID;
    area_C4 = Playground::INVALID;
    area_B3 = Playground::INVALID;

    p_ = nullptr;
    // initPlayground() n'est plus appele dans le constructeur.
    // Il doit etre appele explicitement par le match (O_State_DecisionMakerIA)
    // ou par chaque test fonctionnel avec son propre playground.

}

void OPOS6UL_IAExtended::initPlayground() {
    //terrain horizontal
    p_ = new SymmetricalPlayground(0.0, 0.0, 3400.0, 2500.0, 0.5, 1.0, 1500.0);

    //bordure terrain horizontal — capture les ids pour marquer permanent
    PlaygroundObjectID borderLeftId, borderRightId, borderBottomId, borderTopId;
    p_->add_rectangle_lower_left(borderLeftId,   0,        0,        129, 2000, 0); //cote gauche
    p_->add_rectangle_lower_left(borderRightId,  3000,     0,       -129, 2000, 0); //cote droit
    p_->add_rectangle_lower_left(borderBottomId, 0,        0,       3000,  129, 0); //bas
    p_->add_rectangle_lower_left(borderTopId,    0, 2000-129, 3000, 2000, 0); //haut
    // Bordures = zones FIXES du jeu, jamais desactivables au match.
    // Le flag is_permanent permet a StrategyJsonRunner::validateAgainstPlayground
    // de detecter au chargement qu'une cible de strategie tombe dans une bordure
    // (= cible jamais atteignable) et d'aborter avant le match.
    p_->set_permanent(borderLeftId,   true);
    p_->set_permanent(borderRightId,  true);
    p_->set_permanent(borderBottomId, true);
    p_->set_permanent(borderTopId,    true);
    //bordure terrain vertical
//    p_->add_rectangle_lower_left(0, 0, 129, 3000, 0); //cote gauche
//    p_->add_rectangle_lower_left(2000, 0, -129, 3000, 0); //cote droit
//    p_->add_rectangle_lower_left(0, 0, 2000, 129, 0); //bas
//    p_->add_rectangle_lower_left(0, 3000 - 129, 2000, 3000, 0); //haut


    //bordure terrain lego
//    p_->add_rectangle(1500, 0, 3000, 140, 0); //bottom
//    p_->add_rectangle(1500, 2000, 3000, 140, 0); //top
//    p_->add_rectangle(0, 1000, 140, 2000, 0); //left
//    p_->add_rectangle(3000, 1000, 140, 2000, 0); //right


    //2024
    //p_->add_rectangle(500, 800, 380, 380, 0);


//2023

//
////zone pour eviter le petit robot
//    p_->add_rectangle_lower_left(500, 500, 350, 350, 0);
//
//    //zone bleu adv A3
//    p_->add_rectangle_lower_left(area_A3, 0, 800, 558, 650, 0);
//    //zone bleu adv D4
//    p_->add_rectangle_lower_left(1445, 1550, 558, 650, 0);
//
//    //marron C3
//    p_->add_rectangle(1275, 1125, 380, 380, 0);
//    //marron B3
//    p_->add_rectangle(area_B3, 725, 1125, 340, 340, 0);
//
//    //marron C4
//    p_->add_rectangle(area_C4, 1275, 1875, 380, 380, 0);
//
//    //marron B4
//    p_->add_rectangle(this->area_B4, 725, 1875, 340, 340, 0);
//




    //p_->add_rectangle_lower_left_symmetrical(this->area_start_green, this->area_start_blue, 1400.0, 0.0, 650.0, 650.0, 0.0);
   //p_->add_rectangle_lower_left(this->area_start_green, this->area_start_blue, 1400.0, 0.0, 650.0, 650.0, 0.0);

    //p_->enable(this->area_start_green, false);

//2022
    /*
    //zone de triangle piedestal
    p_->add_rectangle_symmetrical(this->area_triangle_yellow, this->area_triangle_violet, 200.0, 200.0, 300.0, 750.0, M_PI / 4.0);

    //zone aleatoire
    p_->add_rectangle_lower_left_symmetrical(this->area_alea_yellow, this->area_alea_violet, 700.0, 350.0, 550.0, 550.0, 0);

    //zone 3 hexagones devant depart
    //p_->add_rectangle_lower_left_symmetrical(this->area_3_start_yellow, this->area_3_start_violet, 700.0, 1000.0, 430.0, 640.0, 0.0);
    p_->add_rectangle_symmetrical(this->area_3_start_yellow, this->area_3_start_violet, 900.0, 1350.0, 400.0, 400.0, M_PI / 4.0);

    //distrib
    p_->add_rectangle_symmetrical(this->area_distrib_yellow, this->area_distrib_violet, 100.0, 750.0, 200.0, 300.0, 0.0);

    //galerie
    p_->add_rectangle_lower_left_symmetrical(this->area_galerie_yellow, this->area_galerie_violet, 400.0, 1800.0, 1100.0, 200.0, 0.0);
    //barre milieu
    p_->add_rectangle(1500, 1850, 260, 550, 0);
*/


    /*
     std::vector<Point*> relative_point_refs = std::vector<Point*>(3);
     Point p1={0,2000};
     Point p2={3000,2000};
     Point p3={3000,0};

     relative_point_refs[0] = &(p1);
     relative_point_refs[1] = &(p2);
     relative_point_refs[2] = &(p3);
     p_->add_convex_body(0.0, 0.0, relative_point_refs, 0);
     */
    /*
     //zone aleatoire
     p_->add_rectangle_lower_left_symmetrical(this->area_alea_yellow, this->area_alea_blue, 1000.0, 0.0, 500.0, 30.0, 0.0);

     //zone depart
     p_->add_circle_symmetrical(this->area_v_vert_gauche_yellow, this->area_v_vert_gauche_blue, 450, 510, 210, 5);
     p_->add_circle_symmetrical(this->area_v_rouge_droite_yellow, this->area_v_rouge_droite_blue, 450, 1080, 200, 4);
     p_->add_circle_symmetrical(this->area_v_vert_droite_zoneNS_yellow, this->area_v_vert_droite_zoneNS_blue, 300, 400, 200, 4);
     p_->add_circle_symmetrical(this->area_v_rouge_gauche_zoneSN_yellow, this->area_v_rouge_gauche_zoneSN_blue, 300, 1200, 180, 4);

     //centre terrain
     //p_->add_circle_symmetrical(this->area_v_vert_devant_yellow, this->area_v_vert_devant_blue, 1100, 800, 200, 5);
     p_->add_circle_symmetrical(this->area_v_rouge_devant_droite_yellow, this->area_v_rouge_devant_droite_blue, 950, 400, 200, 5);
     p_->add_circle_symmetrical(this->area_v_rouge_devant_gauche_milieu_terrain_yellow, this->area_v_rouge_devant_gouche_milieu_terrain_blue, 1270, 1200, 200, 5);

     */

    //opponent
    p_->add_circle(this->opponent_1, -100.0, -100.0, 350.0, 8);
//    p_->add_circle(this->opponent_2, -100.0, -100.0, 350.0, 8);
//    p_->add_circle(this->opponent_3, -100.0, -100.0, 350.0, 8);
//    p_->add_circle(this->opponent_4, -100.0, -100.0, 350.0, 8);

    // Zone dynamique de test : declaree pour que les edges intersectants
    // soient calcules par compute_edges. DESACTIVEE par defaut, n'a
    // aucun impact en production. O_StrategyJsonRunnerTest l'active
    // ponctuellement pour le scenario SR09 (cible PATH_TO bloquee
    // -> A* renvoie cost=0 -> outcome SK_IMPS).
    p_->add_rectangle_lower_left(this->area_test_blocker, 1750, 950, 100, 100, 0);
    p_->enable(this->area_test_blocker, false);

    // Grenier : element fixe du jeu, centre sur l'axe X du terrain (1500),
    // enabled + permanent au boot. Taille 1814 x 590.
    p_->add_rectangle(this->area_zone_grenier, 1500, 590 / 2, 1814, 590, 0);
    p_->enable(this->area_zone_grenier, true);
    p_->set_permanent(this->area_zone_grenier, true);
    iap_.registerNamedObstacle("zone_grenier", this->area_zone_grenier);

    // ----- Zones de prise (symetriques) : enabled au boot, desactivables -----
    // Le JSON utilise le nom "bleu" ; en match jaune IAbyPath redirige
    // automatiquement vers l'id miroir cote jaune.
    PlaygroundObjectID id_P1_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_P1, id_P1_sym, 175, 800, 450, 500, 0);
    p_->enable(this->area_zone_P1, true);
    p_->enable(id_P1_sym, true);
    iap_.registerSymmetricalObstacle("zone_P1", this->area_zone_P1, id_P1_sym);

    PlaygroundObjectID id_P2_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_P2, id_P2_sym, 175, 1600, 450, 500, 0);
    p_->enable(this->area_zone_P2, true);
    p_->enable(id_P2_sym, true);
    iap_.registerSymmetricalObstacle("zone_P2", this->area_zone_P2, id_P2_sym);

    PlaygroundObjectID id_P3_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_P3, id_P3_sym, 1100, 1825, 500, 450, 0);
    p_->enable(this->area_zone_P3, true);
    p_->enable(id_P3_sym, true);
    iap_.registerSymmetricalObstacle("zone_P3", this->area_zone_P3, id_P3_sym);

    PlaygroundObjectID id_P4_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_P4, id_P4_sym, 1150, 1200, 500, 450, 0);
    p_->enable(this->area_zone_P4, true);
    p_->enable(id_P4_sym, true);
    iap_.registerSymmetricalObstacle("zone_P4", this->area_zone_P4, id_P4_sym);

    // ----- Zones de depose : disabled au boot, activables via JSON ADD_ZONE ---
    // GM2/GM3/GM4 sont symetriques (id stocke = bleu, jaune capture en local).
    PlaygroundObjectID id_GM2_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_GM2, id_GM2_sym, 175, 1200, 450, 500, 0);
    p_->enable(this->area_zone_GM2, false);
    p_->enable(id_GM2_sym, false);
    iap_.registerSymmetricalObstacle("zone_GM2", this->area_zone_GM2, id_GM2_sym);

    PlaygroundObjectID id_GM3_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_GM3, id_GM3_sym, 800, 1200, 500, 450, 0);
    p_->enable(this->area_zone_GM3, false);
    p_->enable(id_GM3_sym, false);
    iap_.registerSymmetricalObstacle("zone_GM3", this->area_zone_GM3, id_GM3_sym);

    PlaygroundObjectID id_GM4_sym = Playground::INVALID;
    p_->add_rectangle_symmetrical(this->area_zone_GM4, id_GM4_sym, 700, 1825, 500, 450, 0);
    p_->enable(this->area_zone_GM4, false);
    p_->enable(id_GM4_sym, false);
    iap_.registerSymmetricalObstacle("zone_GM4", this->area_zone_GM4, id_GM4_sym);

    // GM5 et GM6 sur l'axe central x=1500 : pas de miroir, simple registerNamed.
    p_->add_rectangle(this->area_zone_GM5, 1500, 1825, 500, 450, 0);
    p_->enable(this->area_zone_GM5, false);
    iap_.registerNamedObstacle("zone_GM5", this->area_zone_GM5);

    p_->add_rectangle(this->area_zone_GM6, 1500, 1200, 500, 450, 0);
    p_->enable(this->area_zone_GM6, false);
    iap_.registerNamedObstacle("zone_GM6", this->area_zone_GM6);

    p_->compute_edges();
    iap_.addPlayground(p_);
    iap_.toSVG();

}
void OPOS6UL_IAExtended::move_adv(float x_mm, float y_mm) {
    //p->move(me, 200.0, 0.0)->synchronize();
    //p_->movexy(this->opponent_1, x_mm, y_mm)->synchronize();
}
