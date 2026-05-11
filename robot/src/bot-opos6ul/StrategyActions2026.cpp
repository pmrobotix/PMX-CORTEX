/*!
 * \file
 * \brief Enregistrement des actions et zones pour la Coupe de France 2026.
 *
 * Contient :
 *  - `registerStrategyActions2026`    : mapping JSON action_id -> methodes materielles.
 *  - `setupActivitiesZone2026`        : declaration des zones ia_createZone + ia_addAction.
 *  - les callbacks d'action de zone   : `end_of_match_top`, `push_prise_bas` (statics).
 *
 * Tout est regroupe ici pour que la migration annee+1 se limite a dupliquer
 * ce fichier et adapter aux nouvelles regles.
 */

#include "StrategyActions2026.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "navigator/Navigator.hpp"
#include "navigator/RetryPolicy.hpp"
#include "utils/Chronometer.hpp"

#include "OPOS6UL_ActionsExtended.hpp"
#include "OPOS6UL_IAExtended.hpp"
#include "OPOS6UL_RobotExtended.hpp"

namespace {

const logs::Logger& logger()
{
    static const logs::Logger& instance = logs::LoggerFactory::logger("StrategyActions2026");
    return instance;
}

// =============================================================================
// Callbacks de zone (appeles via IAbyPath::ia_addAction)
// =============================================================================

/*!
 * \brief Action de fin de match : rejoint la zone de fin en haut de table.
 *
 * Attend la fin du chrono (96s), puis avance pour marquer les points de presence.
 * \return true si termine correctement, false si collision non resolue.
 *
 * \deprecated Conservee pour reference. Plus utilisee en 2026 (pas de
 *             "zone_end_top" dans les strategies courantes). A supprimer
 *             apres validation des nouvelles manipulations.
 */
bool end_of_match_top()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    logger().info() << __FUNCTION__ << logs::end;
    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.lastAction(true);
    robot.asserv().setMaxSpeed(true, 40);
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);

    logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;

    robot.ia().iAbyPath().goToZone("zone_end_top", &zone);
    robot.displayPoints();

    Navigator nav(&robot, &robot.ia().iAbyPath());

    logger().info() << __FUNCTION__ << " start zone_end_top x=" << zone.x << " y=" << zone.y << logs::end;
    ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), RetryPolicy::patient());
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " zone_end_top  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return false;
    }
    robot.svgPrintPosition();

    // attente de 96 sec
    while (robot.chrono().getElapsedTimeInSec() <= 96) {
        utils::sleep_for_secs(1);
    }

    ts = nav.line(451);
    robot.svgPrintPosition();

    robot.points += 20;
    robot.displayPoints();
    robot.svgPrintPosition();

    return true;
}

/*!
 * \brief Action de poussee de la prise en zone basse.
 *
 * Sequence composite : ralentit, ignore la detection, rejoint zone_prise_bas,
 * pousse jusqu'a (775, 200). Exposee comme MANIPULATION (`action_id:
 * "push_prise_bas"`) dans le JSON de strategie.
 *
 * \return true si termine, false si collision non resolue (remontee au runner
 *         qui abort l'instruction courante).
 *
 * \deprecated Remplacee par push_elements_zone (manip generique parametrable
 *             par config beacon). Conservee pour reference, plus appelee dans
 *             les strategies courantes. A supprimer apres validation 2026.
 */
bool push_prise_bas()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    logger().info() << __FUNCTION__ << logs::end;
    TRAJ_STATE ts = TRAJ_IDLE;
    ROBOTPOSITION zone;

    robot.asserv().setMaxSpeed(true, 40);
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, true, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, true, true);
    logger().info() << __FUNCTION__ << " start push_prise_bas x=" << zone.x << " y=" << zone.y << logs::end;
    robot.ia().iAbyPath().goToZone("zone_prise_bas", &zone);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    RetryPolicy policyPrise = { 1000000, 30, 30, 0, 0, false };

    ts = nav.moveForwardToAndRotateAbsDeg(zone.x, zone.y, radToDeg(zone.theta), policyPrise);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " zone_prise_bas  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return false;
    }
    robot.svgPrintPosition();

    RetryPolicy policyPush = { 1000000, 10, 10, 0, 0, false };
    ts = nav.moveForwardTo(775, 200, policyPush);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " 775, 200  ===== PB COLLISION FINALE - Que fait-on? ts=" << ts
                         << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        return true;
    }
    robot.svgPrintPosition();
    return true;
}

// =============================================================================
// Push elements de jeu (distance fonction de la config beacon LCD tactile)
// =============================================================================

// Distance de recul de degagement apres la pousse (mm).
constexpr float D_RETREAT = 200.0f;

// Distance de base de la pousse (mm). La fonction push_elements_zone() avance
// de D_BASE + dist_signed[idx] + zoneOffsetFor(zone).
//   - D_BASE par defaut         : 400 mm
//   - P4 et P14 (kZoneOffset)   : -50 mm -> effectif 350 mm
constexpr float D_BASE = 400.0f;

// Offsets specifiques par zone (mm), ajoutes au D_BASE pour les zones avec
// une distance de prise/depose differente de la moyenne. Garde-fou : si
// D_BASE + distBase + offset <= 0, la manip log une erreur et abort.
constexpr float kZoneOffset_P4  = -50.0f;
constexpr float kZoneOffset_P14 = -50.0f;

// Tables indexees sur l'ordre balise (idx 0..5) :
//   0=BBYY  1=YYBB  2=BYYB  3=YBBY  4=BYBY  5=YBYB
//
// Convention :
//   distDirecte = utilisee par les wrappers _H/_G (robot arrive cote DEBUT lecture)
//   distInverse = utilisee par les wrappers _B/_D (robot arrive cote FIN lecture)
//
// Convention couleur (strategie JSON ecrite EN BLEU). En YELLOW,
// push_elements_zone() applique 2 transformations via
// push_elements_test_api::computeDistance() :
//   1) idx pickup -> SWAP_COLOR_IDX[idx] (paires symetriques 0<->1, 2<->3, 4<->5)
//   2) suffixe horizontal _D<->_G (zones P3/P4/P13/P14)
// Cf robot/md/PUSH_ELEMENTS_2026.md.
//
// Valeurs signees (mm) - PLACEHOLDER A CALIBRER sur table.

static constexpr float distDirecte[6] = {
    /* [0] BBYY */  125.0f,
    /* [1] YYBB */ -125.0f,
    /* [2] BYYB */  175.0f,
    /* [3] YBBY */   75.0f,
    /* [4] BYBY */   75.0f,
    /* [5] YBYB */  -75.0f,
};

static constexpr float distInverse[6] = {
    /* [0] BBYY */ -125.0f,
    /* [1] YYBB */  125.0f,
    /* [2] BYYB */  175.0f,
    /* [3] YBBY */   75.0f,
    /* [4] BYBY */  -75.0f,
    /* [5] YBYB */   75.0f,
};

} // namespace (anonymous, fin partielle pour exposer le namespace public)

// =============================================================================
// API exposee push_elements_test_api : utilisee par O_PushElementsTest
// =============================================================================
namespace push_elements_test_api {

const uint8_t SWAP_COLOR_IDX[6] = { 1, 0, 3, 2, 5, 4 };

float distDirecteAt(uint8_t idx)
{
    return (idx < 6) ? distDirecte[idx] : -1.0f;
}

float distInverseAt(uint8_t idx)
{
    return (idx < 6) ? distInverse[idx] : -1.0f;
}

float zoneOffsetFor(const char* zoneName)
{
    if (zoneName == nullptr) return 0.0f;
    if (strcmp(zoneName, "P4")  == 0) return kZoneOffset_P4;
    if (strcmp(zoneName, "P14") == 0) return kZoneOffset_P14;
    return 0.0f;
}

float retreatMm()
{
    return D_RETREAT;
}

float dBaseMm()
{
    return D_BASE;
}

bool isHorizontalZone(const char* zoneName)
{
    if (zoneName == nullptr) return false;
    return (strcmp(zoneName, "P3")  == 0 || strcmp(zoneName, "P4")  == 0
         || strcmp(zoneName, "P13") == 0 || strcmp(zoneName, "P14") == 0);
}

float computeDistance(uint8_t pickupIdx, const char* zoneName,
                      bool sensInverse, bool yellow)
{
    if (pickupIdx > 5) return -1.0f;

    if (yellow) {
        if (isHorizontalZone(zoneName)) sensInverse = !sensInverse;
        pickupIdx = SWAP_COLOR_IDX[pickupIdx];
    }

    const float distBase = sensInverse ? distInverse[pickupIdx] : distDirecte[pickupIdx];
    const float offset   = zoneOffsetFor(zoneName);
    const float dist     = D_BASE + distBase + offset;
    if (dist <= 0.0f) return -1.0f;
    return dist;
}

} // namespace push_elements_test_api

// =============================================================================
// push_elements_zone : la manipulation runtime utilisee par les wrappers
// (registerStrategyActions2026 -> push_elements_P*_*).
// =============================================================================

// Reprise de l'anonymous namespace pour les helpers locaux qui suivent.
namespace {

bool push_elements_zone_impl(uint8_t pickupIdx, const char* zoneName, bool sensInverse)
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    const bool yellow = robot.isMatchColor();

    // Calcul pur de la distance via l'API exposee. Reproduit la logique de
    // swap idx + flip suffixe horiz + offset, sans dependance asserv/sensors.
    const float dist = push_elements_test_api::computeDistance(
        pickupIdx, zoneName, sensInverse, yellow);

    // Re-applique le swap sur idx/sensInverse pour le log (computeDistance
    // les a appliques en interne mais ne les retourne pas).
    uint8_t idxLog = pickupIdx;
    bool    invLog = sensInverse;
    if (yellow) {
        if (push_elements_test_api::isHorizontalZone(zoneName)) invLog = !invLog;
        if (idxLog <= 5) idxLog = push_elements_test_api::SWAP_COLOR_IDX[idxLog];
    }

    if (dist < 0.0f) {
        logger().error() << "push_elements_zone " << zoneName
                         << " idx=" << (int)pickupIdx
                         << " sens=" << (sensInverse ? "INVERSE" : "DIRECTE")
                         << (yellow ? " (YELLOW)" : "")
                         << " -> dist invalide, abort" << logs::end;
        return false;
    }

    const char* sensLabel = invLog ? "INVERSE" : "DIRECTE";
    logger().info() << "push_elements_zone " << zoneName
                    << " idx=" << (int)idxLog << " sens=" << sensLabel
                    << (yellow ? " (YELLOW post-swap)" : "") << logs::end;

    const float distBase = invLog ? distInverse[idxLog] : distDirecte[idxLog];
    const float offset   = push_elements_test_api::zoneOffsetFor(zoneName);
    logger().info() << "push_elements_zone " << zoneName << " avance=" << dist
                    << "mm (D_BASE=" << D_BASE << " + dist[idx]=" << distBase
                    << " + offset=" << offset
                    << ") puis recul=" << D_RETREAT << "mm" << logs::end;

    // Sauvegarde du cap vitesse user (rest. apres manip pour ne pas laisser
    // 20% pour les tasks suivantes du JSON).
    const bool prevSpeedActive = robot.asserv().getUserMaxSpeedActive();
    const int  prevSpeedPct    = robot.asserv().getUserMaxSpeedPercent();
    auto restoreSpeed = [&] { robot.asserv().setMaxSpeed(prevSpeedActive, prevSpeedPct); };

    // Vitesse reduite pour la pousse (couple max, eviter de balayer les pieces).
    robot.asserv().setMaxSpeed(true, 20);
    // Front center actif (collision sur element pousse), front lateral + back ignores.
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, false, true);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    RetryPolicy policyPush = RetryPolicy::standard();   // 2 retries obstacle/collision

    TRAJ_STATE ts = nav.line(dist, policyPush);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " " << zoneName << " avance ts=" << ts << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        restoreSpeed();
        return false;
    }
    robot.svgPrintPosition();

    ts = nav.line(-D_RETREAT, policyPush);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " " << zoneName << " recul ts=" << ts << logs::end;
        robot.asserv().resetEmergencyOnTraj();
    }
    robot.svgPrintPosition();
    restoreSpeed();
    return true;
}

// =============================================================================
// Declaration des zones (ia_createZone + ia_addAction)
// =============================================================================

// =============================================================================
// Curseur : deploie le bon bras selon couleur match, avance, replie les bras.
// STUB - a finaliser avec servos (calibration distance + recul + cote bleu/jaune).
// =============================================================================
bool curseur_zone()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    const bool yellow = robot.isMatchColor();
    logger().info() << "curseur_zone STUB yellow=" << yellow << logs::end;

    // TODO (a valider/calibrer demain) :
    //  1) Sauvegarde vitesse user, passe a 20% (cf push_elements_zone_impl).
    //  2) Ignore front center capteurs (la pousse curseur va toucher l'element).
    //  3) Choix du bras selon couleur :
    //       - BLEU   -> ax12_bras_droit() ou ax12_bras_gauche()  [a trancher]
    //       - JAUNE  -> l'autre cote
    //     keep=0 (maintien), eta=400ms.
    //  4) nav.line(D_CURSEUR)  - placeholder D_CURSEUR = 400.0f (a calculer).
    //  5) Recul ? nav.line(-D_RETREAT) optionnel.
    //  6) Replie les DEUX bras : ax12_bras_droit_init() + ax12_bras_gauche_init().
    //  7) Restore vitesse user.

    return true;
}

// =============================================================================
// Defense : si adv dans une zone d'attaque, va au point de defense correspondant,
// attend 5s dos a l'adv, puis rend la main (PATH_TO de la task suivante s'occupe
// du retour). No-op si adv hors zone ou non detecte.
//
// Coords ECRITES EN BLEU (convention single-color), miroir auto jaune :
//   - X : `robot.changeMatchX(x_bleu)` (retourne x ou 3000-x)
//   - Y : pas de miroir (symetrie X uniquement)
//   - theta_def : 180 deg BLEU / 0 deg JAUNE (dos a l'adv)
//
// Zones d'attaque adv et points de defense (BLEU) :
//   Zone 1 : adv dans rect [1700..2000] x [1300..1800]  -> Defense (1500, 1500)
//   Zone 2 : adv dans rect [1700..2000] x [ 700..1100]  -> Defense (1500,  900)
//
// STUB - mouvement desactive tant que la plomberie adv position n'est pas finie.
// Plomberie manquante : Asserv::adv_pos_centre_ n'est jamais set ; soit cabler
// depuis Sensors (transform robot->table + setter), soit faire le transform a
// la volee depuis sensors().getPositionsAdv() (repere ROBOT) dans cette action.
// =============================================================================
bool defense_if_needed()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    const bool yellow = robot.isMatchColor();

    // Coords BLEU (a calibrer/affiner demain si besoin)
    constexpr float ZONE1_XMIN_B = 1700.0f, ZONE1_XMAX_B = 2000.0f;
    constexpr float ZONE1_YMIN_B = 1300.0f, ZONE1_YMAX_B = 1800.0f;
    constexpr float DEF1_X_B = 1500.0f,     DEF1_Y_B = 1500.0f;

    constexpr float ZONE2_XMIN_B = 1700.0f, ZONE2_XMAX_B = 2000.0f;
    constexpr float ZONE2_YMIN_B =  700.0f, ZONE2_YMAX_B = 1100.0f;
    constexpr float DEF2_X_B = 1500.0f,     DEF2_Y_B =  900.0f;

    // (void) pour silence warnings tant que le corps est un stub
    (void) ZONE1_XMIN_B; (void) ZONE1_XMAX_B; (void) ZONE1_YMIN_B; (void) ZONE1_YMAX_B;
    (void) DEF1_X_B;     (void) DEF1_Y_B;
    (void) ZONE2_XMIN_B; (void) ZONE2_XMAX_B; (void) ZONE2_YMIN_B; (void) ZONE2_YMAX_B;
    (void) DEF2_X_B;     (void) DEF2_Y_B;

    // Position adv en repere TABLE (mm) - cf Asserv::pos_getAdvPosition()
    // ATTENTION : adv_pos_centre_ n'est jamais set ailleurs (-100,-100 init), donc
    // ce stub log juste la valeur ; aucune detection reelle tant que la plomberie
    // n'est pas finie (cf bloc commentaire en tete de fonction).
    ROBOTPOSITION adv = robot.asserv().pos_getAdvPosition();
    logger().info() << "defense_if_needed STUB yellow=" << yellow
                    << " adv=(" << adv.x << "," << adv.y << ")" << logs::end;

    // TODO (a finaliser demain) :
    //  1) Cabler adv position : option a) Sensors::onTimer remplit
    //     asserv.adv_pos_centre_ via un setter + transform robot->table.
    //     option b) ici, lire sensors().getPositionsAdv() (repere ROBOT) +
    //     transformer avec pos courante du robot.
    //
    //  2) Si adv invalide (x<0 || y<0 ou flag absent) -> return true no-op.
    //
    //  3) Calcul des rectangles en repere REEL (apres miroir X) :
    //       float z1xmin = std::min(robot.changeMatchX(ZONE1_XMIN_B),
    //                               robot.changeMatchX(ZONE1_XMAX_B));
    //       float z1xmax = std::max(... idem ...);
    //       (Y inchange : ZONE1_YMIN_B / ZONE1_YMAX_B directement)
    //     idem zone 2.
    //
    //  4) Test inclusion :
    //       bool in_z1 = adv.x>=z1xmin && adv.x<=z1xmax && adv.y>=z1ymin && adv.y<=z1ymax;
    //       bool in_z2 = ... ;
    //
    //  5) Si in_z1 ou in_z2 :
    //       float def_x = robot.changeMatchX(in_z1 ? DEF1_X_B : DEF2_X_B);
    //       float def_y =                    in_z1 ? DEF1_Y_B : DEF2_Y_B;
    //       float theta_def_deg = yellow ? 0.0f : 180.0f;
    //       Navigator nav(&robot, &robot.ia().iAbyPath());
    //       nav.moveForwardToAndRotateAbsDeg(def_x, def_y, theta_def_deg);
    //       utils::sleep_for_secs(5);
    //     Sinon : no-op.
    //
    //  6) return true; (runner enchaine sur task suivante, PATH_TO gere le repli)
    //
    //  V2 (plus tard) : ajouter test vecteur d'approche pour eviter le detour
    //  quand l'adv est immobile ou s'eloigne (cf option A double-echantillon
    //  300ms + test vx selon couleur).

    return true;
}

void setupZonesHomologation(OPOS6UL_RobotExtended& robot)
{
    logger().info() << "setupActivitiesZone2026 : strategy=all (homologation)" << logs::end;
    logger().debug() << "color = " << robot.getMyColor() << logs::end;

    robot.ia().iAbyPath().ia_createZone("zone_end_top",    150, 1550, 450, 450, 350, 1100,  90);
    robot.ia().iAbyPath().ia_createZone("zone_start",     1000,    0, 450, 450, 1300, 400,  90);
    robot.ia().iAbyPath().ia_createZone("zone_prise_bas",  550,    0, 450, 100,  775, 550, -90);

    robot.ia().iAbyPath().ia_addAction("end_of_match_top", &end_of_match_top);
}

void setupZonesTableTest(OPOS6UL_RobotExtended& robot)
{
    logger().info() << "setupActivitiesZone2026 : strategy=tabletest (layout decale -420mm)"
                    << logs::end;
    logger().debug() << "color = " << robot.getMyColor() << logs::end;

    robot.ia().iAbyPath().ia_createZone("zone_end_top",    150, 1550 - 420, 450, 450,  350, 1100 - 420,  90);
    robot.ia().iAbyPath().ia_createZone("zone_start",     1000,         0,  450, 450, 1300,        400,  90);
    robot.ia().iAbyPath().ia_createZone("zone_prise_bas",  550,         0,  450, 100,  775,        550, -90);

    robot.ia().iAbyPath().ia_addAction("end_of_match_top", &end_of_match_top);
}

} // namespace

// =============================================================================
// API publique
// =============================================================================

// Thin wrapper public : delegue a push_elements_zone_impl (anonymous namespace).
// Permet au test O_PushElementsTest d'invoquer la manip directement.
bool push_elements_zone(uint8_t pickupIdx, const char* zoneName, bool sensInverse)
{
    return push_elements_zone_impl(pickupIdx, zoneName, sensInverse);
}

void setupActivitiesZone2026(OPOS6UL_RobotExtended& robot, const std::string& strategy)
{
    if (strategy == "tabletest") {
        setupZonesTableTest(robot);
    } else if (strategy == "all") {
        setupZonesHomologation(robot);
    } else {
        logger().error() << "strategy '" << strategy << "' inconnue (attendu: all|tabletest)"
                         << logs::end;
    }
}

void registerStrategyActions2026(ActionRegistry& registry, OPOS6UL_RobotExtended& robot)
{
    // --- Banderole (action de fin de match) ---
    registry.registerAction("banderole",
        [&robot]() { robot.actions().ax12_GO_banderole(); return true; });
    registry.registerAction("banderole_init",
        [&robot]() { robot.actions().ax12_init_banderole(); return true; });

    // --- Bras lateraux ---
    registry.registerAction("bras_droit",
        [&robot]() { robot.actions().ax12_bras_droit(); return true; });
    registry.registerAction("bras_droit_init",
        [&robot]() { robot.actions().ax12_bras_droit_init(); return true; });
    registry.registerAction("bras_gauche",
        [&robot]() { robot.actions().ax12_bras_gauche(); return true; });
    registry.registerAction("bras_gauche_init",
        [&robot]() { robot.actions().ax12_bras_gauche_init(); return true; });

    // --- Reset global des AX12 ---
    registry.registerAction("init_all",
        [&robot]() { robot.actions().ax12_init(); return true; });

    // --- Curseur (STUB - a finaliser demain avec servos + calibration) ---
    registry.registerAction("curseur",
        [](){ return curseur_zone(); });

    // --- Defense (STUB - a finaliser demain avec rectangles zones + coords points) ---
    registry.registerAction("defense_if_needed",
        [](){ return defense_if_needed(); });

    // --- Sequences composites (deplacement + manipulation) ---
    // Les actions ci-dessous combinent mouvement + reglages capteurs + retry
    // specifique. Plus simple en C++ qu'en enchainement de tasks JSON
    // (surtout pour les cas "push" qui desactivent toute la detection).
    registry.registerAction("push_prise_bas",
        [](){ return push_prise_bas(); });

    // --- Push elements : 16 entrees (8 zones x 2 sens) ---
    // Convention de suffixe = COTE D'ARRIVEE du robot dans la zone :
    //   _B = robot arrive PAR LE BAS, avance vers le haut (Y+)    - INVERSE
    //   _H = robot arrive PAR LE HAUT, avance vers le bas (Y=0)   - DIRECTE
    //   _D = robot arrive PAR LA DROITE, avance vers la gauche    - INVERSE
    //   _G = robot arrive PAR LA GAUCHE, avance vers la droite    - DIRECTE
    //
    // Sens DIRECTE = robot avance dans le sens de lecture balise (haut->bas
    // vertical, gauche->droite horizontal) = utilise distDirecte.
    // Sens INVERSE = oppose a la lecture balise = utilise distInverse.
    //
    // Chaque action lit pickup_P{N} (idx 0..5) et delegue a push_elements_zone.
    // Cf robot/md/PUSH_ELEMENTS_2026.md.

    // Verticales (P1, P2, P11, P12) : _B = arrive du bas (INVERSE), _H = arrive du haut (DIRECTE)
    registry.registerAction("push_elements_P1_B",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP1(),  "P1",  true);  });
    registry.registerAction("push_elements_P1_H",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP1(),  "P1",  false); });
    registry.registerAction("push_elements_P2_B",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP2(),  "P2",  true);  });
    registry.registerAction("push_elements_P2_H",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP2(),  "P2",  false); });
    registry.registerAction("push_elements_P11_B",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP11(), "P11", true);  });
    registry.registerAction("push_elements_P11_H",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP11(), "P11", false); });
    registry.registerAction("push_elements_P12_B",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP12(), "P12", true);  });
    registry.registerAction("push_elements_P12_H",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP12(), "P12", false); });

    // Horizontales (P3, P4, P13, P14) : _D = arrive de droite (INVERSE), _G = arrive de gauche (DIRECTE)
    registry.registerAction("push_elements_P3_D",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP3(),  "P3",  true);  });
    registry.registerAction("push_elements_P3_G",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP3(),  "P3",  false); });
    registry.registerAction("push_elements_P4_D",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP4(),  "P4",  true);  });
    registry.registerAction("push_elements_P4_G",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP4(),  "P4",  false); });
    registry.registerAction("push_elements_P13_D",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP13(), "P13", true);  });
    registry.registerAction("push_elements_P13_G",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP13(), "P13", false); });
    registry.registerAction("push_elements_P14_D",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP14(), "P14", true);  });
    registry.registerAction("push_elements_P14_G",
        [](){ return push_elements_zone(OPOS6UL_RobotExtended::instance().pickupP14(), "P14", false); });


}
