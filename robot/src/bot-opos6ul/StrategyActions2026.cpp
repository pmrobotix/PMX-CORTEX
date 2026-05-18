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
#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>

#include "action/Sensors.hpp"
#include "asserv/Asserv.hpp"
#include "ia/ActionRegistry.hpp"
#include "ia/IAbyPath.hpp"
#include "interface/AAsservDriver.hpp"
#include "log/LoggerFactory.hpp"
#include "log/SvgWriter.hpp"
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
// Visualisation SVG : 4 rectangles places relativement au robot
// =============================================================================
//
// L'idx represente la lecture balise (= match mode) : independant de la position
// du robot. Convention :
//   - Verticales   : char[0] est en HAUT (Y+), char[3] en BAS (Y-)
//   - Horizontales : char[0] est a GAUCHE (X-), char[3] a DROITE (X+)
//
// Suivant le cap du robot, char[0] est plus proche ou plus loin :
//   - cap dans le sens lecture (Y- vertical, X+ horizontal) : char[0] proche
//   - cap a contre-sens (Y+, X-)                            : char[3] proche

// Dimensions d'un rectangle (mm).
//   short = profondeur dans le sens de pousse (forward)
//   long  = largeur perpendiculaire
constexpr float kRectLong         = 150.0f;
constexpr float kRectShort        =  50.0f;
// Distance du centre robot a la face octogonale avant (mm).
constexpr float kRobotFrontOffset = 115.0f;
// Distance du centre robot a la face octogonale arriere (mm).
// Utilise par push_elements_zone(backward=true) pour le placement des rects SVG.
constexpr float kRobotRearOffset = 130.0f;
// Convention 2026 : le centre robot est place a 200mm de la face proche de
// l'element 1, peu importe la direction de pousse (forward ou backward).
// Choix derive : rayon de rotation effectif ~160mm (rayon SVG 140 + marge
// protuberances 20) + 40mm de safety user = 200mm centre -> element 1.
// Cette constante est utilisee par drawConfigAtPose pour placer la viz SVG
// des 4 rects (independant de forward/backward : meme position physique).
constexpr float kCubeClosestFromCenterMm = 200.0f;
// Gap (mm) entre la face avant (115mm du centre) et l'element 1 = 200 - 115.
// Utilise par push_elements_zone_impl pour calculer le push physique reel
// (= robot_displacement - gap, car les 85mm initiaux sont traverses sans
// contact avec les cubes). En backward, le gap est 70mm (= 200 - 130), mais
// avec kBackwardPushAdjustmentMm = -15 sur effectiveDist, on retombe sur le
// meme push physique = dist - kStartGapMm dans les deux modes.
constexpr float kStartGapMm = kCubeClosestFromCenterMm - kRobotFrontOffset;  // 85
// Ajustement geometrique applique a la distance brute en mode backward (mm).
// Pas empirique : c'est exactement kRobotFrontOffset - kRobotRearOffset = -15.
// En backward, la face arriere est 15mm plus eloignee du centre que la face
// avant en forward, donc la face arriere est deja 15mm plus proche des
// elements quand le robot est place au meme X/Y -> 15mm de moins a parcourir.
constexpr float kBackwardPushAdjustmentMm = -15.0f;

// Sequence des 4 couleurs par idx balise.
//   0=BBYY 1=YYBB 2=BYYB 3=YBBY 4=BYBY 5=YBYB
constexpr const char* kIdxToSequence[6] = {
    "BBYY", "YYBB", "BYYB", "YBBY", "BYBY", "YBYB"
};

// Retourne true si char[0] doit etre cote robot (le plus proche).
//   Vertical-dominant (|sin| > |cos|) : sin < 0 => cap Y-, dans sens lecture.
//   Horizontal-dominant                : cos > 0 => cap X+, dans sens lecture.
//
// backward=true : la "direction de pousse" est l'inverse du cap robot, donc
// on flippe cos/sin (equivalent a tester theta + pi).
bool char0IsClosestToRobot(float theta_rad, bool backward = false)
{
    float c = std::cos(theta_rad);
    float s = std::sin(theta_rad);
    if (backward) { c = -c; s = -s; }
    if (std::fabs(s) > std::fabs(c)) {
        return s < 0.0f;          // vertical : lecture Y-
    }
    return c > 0.0f;              // horizontal : lecture X+
}

// Dessine la config (4 rectangles) devant le robot, a la pose donnee, decalee
// en avant de `extraForward` mm (0 pour position initiale, `dist` pour finale).
// dashed=false : fill plein. dashed=true : pointilles + transparence.
//
// backward=true : la "direction de pousse" est l'inverse du cap robot. Les
// rects sont places derriere le robot (signDir = -1 sur fx/fy), avec un offset
// initial kRobotRearOffset (130mm) au lieu de kRobotFrontOffset (115mm). Le
// mapping char0/char3 (char0IsClosestToRobot) est aussi inverse via backward.
void drawConfigAtPose(OPOS6UL_RobotExtended& robot,
                      float poseX, float poseY, float poseTheta_rad,
                      uint8_t idx, bool /*yellow*/,
                      float extraForward, bool dashed,
                      bool backward = false)
{
    if (idx > 5) return;

    // L'idx vient de la balise = config physique fixe (char[0] au HAUT/GAUCHE,
    // char[3] au BAS/DROITE). Pas de swap d'affichage selon couleur ou cap.
    const char* seq = kIdxToSequence[idx];

    // signDir : +1 si forward (push vers cap), -1 si backward (push oppose au cap).
    // Pas besoin de faceOffset ici : la convention 2026 place l'element 1 a
    // kCubeClosestFromCenterMm (200mm) du centre robot dans la direction de
    // pousse, peu importe quelle face pousse (avant 115 ou arriere 130).
    const float signDir = backward ? -1.0f : +1.0f;

    // Vecteurs unitaires : forward (sens pousse), left (perpendiculaire +90 deg).
    const float fx = signDir * std::cos(poseTheta_rad);
    const float fy = signDir * std::sin(poseTheta_rad);
    const float lx = -fy;
    const float ly =  fx;

    // Demi-dimensions
    const float hF = kRectShort / 2.0f;   // 25mm dans le sens forward
    const float hL = kRectLong  / 2.0f;   // 75mm dans le sens perpendiculaire

    const bool char0Closest = char0IsClosestToRobot(poseTheta_rad, backward);

    for (int i = 0; i < 4; ++i) {
        // i = position dans la pile depuis le robot (0 = proche, 3 = loin).
        // L'indice de caractere dans la sequence depend du sens de lecture.
        const int  charIdx = char0Closest ? i : (3 - i);
        const char c       = seq[charIdx];

        // Distance dans le sens de pousse du centre du rect[i] depuis le centre robot.
        // Convention 2026 : element 1 face proche a kCubeClosestFromCenterMm
        // du centre dans la direction de pousse (independant forward/backward).
        // Pour i=0 et extraForward=0 -> fwd = 200 + 25 = 225, donc centre rect 0
        // a 225mm du centre robot, face proche a 200mm (= convention).
        const float fwd = kCubeClosestFromCenterMm + extraForward
                        + static_cast<float>(i) * kRectShort + hF;

        // Centre du rect[i] en coords monde.
        const float cxW = poseX + fx * fwd;
        const float cyW = poseY + fy * fwd;

        // 4 coins (centre +/- hF*forward +/- hL*left) en coords monde.
        const float p1x = cxW + hF*fx + hL*lx;   // avant-gauche
        const float p1y = cyW + hF*fy + hL*ly;
        const float p2x = cxW + hF*fx - hL*lx;   // avant-droite
        const float p2y = cyW + hF*fy - hL*ly;
        const float p3x = cxW - hF*fx - hL*lx;   // arriere-droite
        const float p3y = cyW - hF*fy - hL*ly;
        const float p4x = cxW - hF*fx + hL*lx;   // arriere-gauche
        const float p4y = cyW - hF*fy + hL*ly;

        const char* fillColor = (c == 'B' || c == 'b')
            ? "#3060ff"   // bleu
            : "#ffd700";  // jaune

        // SVG : Y flippe (negation du Y monde).
        std::ostringstream pts;
        pts << p1x << "," << -p1y << " "
            << p2x << "," << -p2y << " "
            << p3x << "," << -p3y << " "
            << p4x << "," << -p4y;

        std::ostringstream ss;
        ss << "<polygon points='" << pts.str() << "' fill='" << fillColor << "'"
           << " stroke='black' stroke-width='1'"
           << " fill-opacity='" << (dashed ? "0.35" : "0.7") << "'";
        if (dashed) {
            ss << " stroke-dasharray='6,3'";
        }
        ss << " />";

        robot.svgw().logger().info() << ss.str() << logs::end;
    }
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
    robot.asserv().setMaxSpeed(true, 100);
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

    robot.asserv().setMaxSpeed(true, 100);
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
//   - D_BASE par defaut         : 485 mm  (= ancien 400 calibre pour 0 gap
//                                  + kStartGapMm = 85, traversee du gap face
//                                  avant-element au placement convention 2026)
//   - P4 et P14 (kZoneOffset)   : -50 mm -> effectif 435 mm
// Convention 2026 : le centre robot est place a 200mm de l'element 1, donc
// la face avant est a kStartGapMm = 85mm de l'element en forward. Le robot
// doit traverser ce gap avant de toucher l'element, d'ou le +85 sur D_BASE.
constexpr float D_BASE = 485.0f;

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

// Sentinelle "pas d'override" : valeur volontairement aberrante (1e9 mm) qui
// ne peut jamais etre une distance reelle. Une cellule de kZoneDistOverride
// egale a kNoOverride signifie "utiliser la table partagee distDirecte/Inverse".
constexpr float kNoOverride = 1e9f;

// Table d'override de distance par ZONE PHYSIQUE + config (idx 0..5).
//
// Probleme resolu : distDirecte/distInverse sont PARTAGEES entre toutes les
// zones. Certaines zones ("P inversees" : P4, P13, P14) ont besoin d'une
// valeur dist[idx] propre qui ne peut pas passer par la table partagee sans
// affecter P1/P3. Cette table permet de SURCHARGER le terme dist[idx] pour une
// zone et une config donnees, sans toucher distDirecte/distInverse.
//
// Indexation : l'idx est l'idx POST-SWAP (meme idx que distDirecte/distInverse).
// La cellule idx 2 couvre donc BLEU `/u ALL 2` ET JAUNE `/u ALL 3`
// (SWAP_COLOR_IDX[3]=2). C'est voulu.
//
// Miroir YELLOW : computeDistance() resout la zone physique via mirrorZone()
// avant le lookup. En YELLOW le wrapper `push_elements_P3_*` opere physiquement
// sur P13 (override applique) et `push_elements_P13_*` opere sur P3 (pas
// d'override). Cf robot/md/PUSH_ELEMENTS_2026.md.
//
// Quand une cellule != kNoOverride, sa valeur REMPLACE le terme dist[idx].
struct ZoneDistOverride
{
    const char* zone;     // zone physique (P1..P4, P11..P14)
    float       dist[6];  // override par config idx 0..5 ; kNoOverride = aucun
};

// CALIBRATION : seule la valeur -175 @ idx 2 est calibree sur table reelle.
// Les autres cellules restent a kNoOverride (a calibrer si besoin).
static constexpr ZoneDistOverride kZoneDistOverride[] = {
    { "P4",  { kNoOverride, kNoOverride, -175.0f, kNoOverride, kNoOverride, kNoOverride } },
    { "P13", { kNoOverride, kNoOverride, -175.0f, kNoOverride, kNoOverride, kNoOverride } },
    { "P14", { kNoOverride, kNoOverride, -175.0f, kNoOverride, kNoOverride, kNoOverride } },
};

// Mappe une zone vers sa zone miroir physique : P1<->P11, P2<->P12, P3<->P13,
// P4<->P14. Retourne zoneName inchange si aucun mapping (ou nullptr).
// Modele : le if/else de resolvePickupForZone (meme couplage de zones).
const char* mirrorZone(const char* zoneName)
{
    if (zoneName == nullptr) return zoneName;
    if (strcmp(zoneName, "P1")  == 0) return "P11";
    if (strcmp(zoneName, "P2")  == 0) return "P12";
    if (strcmp(zoneName, "P3")  == 0) return "P13";
    if (strcmp(zoneName, "P4")  == 0) return "P14";
    if (strcmp(zoneName, "P11") == 0) return "P1";
    if (strcmp(zoneName, "P12") == 0) return "P2";
    if (strcmp(zoneName, "P13") == 0) return "P3";
    if (strcmp(zoneName, "P14") == 0) return "P4";
    return zoneName;
}

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

float distOverrideFor(const char* zoneName, uint8_t idx)
{
    if (zoneName == nullptr || idx > 5) return kNoOverride;
    for (const auto& entry : kZoneDistOverride) {
        if (strcmp(zoneName, entry.zone) == 0) return entry.dist[idx];
    }
    return kNoOverride;
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

    // Zone physique reellement traitee : en YELLOW le miroir Asserv place le
    // robot sur la zone miroir de la table (cf mirrorZone / resolvePickupForZone).
    // L'override de distance est keye par zone PHYSIQUE.
    const char* physZone = yellow ? mirrorZone(zoneName) : zoneName;

    // Override de dist[idx] (idx POST-swap). Si une cellule de kZoneDistOverride
    // est definie pour cette zone physique + config, elle remplace le terme
    // distDirecte/distInverse ; sinon on retombe sur la table partagee.
    const float ov       = distOverrideFor(physZone, pickupIdx);
    const float distBase = (ov != kNoOverride)
                         ? ov
                         : (sensInverse ? distInverse[pickupIdx] : distDirecte[pickupIdx]);
    const float offset   = zoneOffsetFor(zoneName);
    const float dist     = D_BASE + distBase + offset;
    if (dist <= 0.0f) return -1.0f;
    return dist;
}

uint8_t resolvePickupForZone(const char* zoneName, bool yellow)
{
    if (zoneName == nullptr) return 0;
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();

    if (strcmp(zoneName, "P1")  == 0) return yellow ? robot.pickupP11() : robot.pickupP1();
    if (strcmp(zoneName, "P2")  == 0) return yellow ? robot.pickupP12() : robot.pickupP2();
    if (strcmp(zoneName, "P3")  == 0) return yellow ? robot.pickupP13() : robot.pickupP3();
    if (strcmp(zoneName, "P4")  == 0) return yellow ? robot.pickupP14() : robot.pickupP4();
    if (strcmp(zoneName, "P11") == 0) return yellow ? robot.pickupP1()  : robot.pickupP11();
    if (strcmp(zoneName, "P12") == 0) return yellow ? robot.pickupP2()  : robot.pickupP12();
    if (strcmp(zoneName, "P13") == 0) return yellow ? robot.pickupP3()  : robot.pickupP13();
    if (strcmp(zoneName, "P14") == 0) return yellow ? robot.pickupP4()  : robot.pickupP14();

    return 0;
}

} // namespace push_elements_test_api

// =============================================================================
// push_elements_zone : la manipulation runtime utilisee par les wrappers
// (registerStrategyActions2026 -> push_elements_P*_*).
// =============================================================================

// Reprise de l'anonymous namespace pour les helpers locaux qui suivent.
namespace {

// Helper pour les lambdas wrappers : lit le pickup_P{N} adapte a la couleur
// match courante du robot. Delegue au coeur logique expose dans le test API
// (push_elements_test_api::resolvePickupForZone) pour que la regle de mapping
// reste testable sans mocker isMatchColor.
//
// Convention couleur (strategie JSON ecrite EN BLEU) : en YELLOW, le miroir
// Asserv place physiquement le robot sur la zone miroir de la table (P1<->P11,
// P2<->P12, P3<->P13, P4<->P14). Le wrapper `push_elements_P{N}_*` doit donc
// lire la config beacon de la zone miroir pour que la sequence physique
// poussee soit bien celle de la zone reelle.
//
// La balise expose 8 configs independantes (pas de swap cote balise, cf
// teensy/IO_t41_ToF_DetectionBeacon/MATCH_CONFIG_UI.md). C'est cote brain
// qu'on aiguille selon la couleur.
//
// 3eme transformation de "Convention couleur" (cf PUSH_ELEMENTS_2026.md), en
// plus du flip suffixe horizontal `_D` <-> `_G` et du `SWAP_COLOR_IDX[idx]`.
uint8_t pickupForZone(const char* zoneName)
{
    const bool yellow = OPOS6UL_RobotExtended::instance().isMatchColor();
    return push_elements_test_api::resolvePickupForZone(zoneName, yellow);
}

bool push_elements_zone_impl(uint8_t pickupIdx, const char* zoneName, bool sensInverse, bool backward)
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
                         << (backward ? " (BACKWARD)" : "")
                         << (yellow ? " (YELLOW)" : "")
                         << " -> dist invalide, abort" << logs::end;
        return false;
    }

    const char* sensLabel = invLog ? "INVERSE" : "DIRECTE";
    logger().info() << "push_elements_zone " << zoneName
                    << " idx=" << (int)idxLog << " sens=" << sensLabel
                    << (backward ? " (BACKWARD)" : "")
                    << (yellow ? " (YELLOW post-swap)" : "") << logs::end;

    // Recompute du terme dist[idx] pour le log, en appliquant l'override sur
    // la zone PHYSIQUE (miroir si yellow) avec l'idx post-swap idxLog, pour
    // rester coherent avec computeDistance().
    const char* physZoneLog = yellow ? mirrorZone(zoneName) : zoneName;
    const float ovLog       = push_elements_test_api::distOverrideFor(physZoneLog, idxLog);
    const bool  hasOverride = (ovLog != kNoOverride);
    const float distBase    = hasOverride
                            ? ovLog
                            : (invLog ? distInverse[idxLog] : distDirecte[idxLog]);
    const float offset      = push_elements_test_api::zoneOffsetFor(zoneName);
    logger().info() << "push_elements_zone " << zoneName << " avance=" << dist
                    << "mm (D_BASE=" << D_BASE << " + dist[idx]=" << distBase
                    << (hasOverride ? " (override)" : "")
                    << " + offset=" << offset
                    << ") puis recul=" << D_RETREAT << "mm" << logs::end;

    // Ajustement geometrique pour backward : on retranche kBackwardPushAdjustmentMm
    // a la distance brute (-15mm = kRobotFrontOffset - kRobotRearOffset).
    // signDir = -1 inverse le sens du nav.line pour pousser arriere.
    const float effectiveDist = backward
        ? (dist + kBackwardPushAdjustmentMm)
        : dist;
    const float signDir = backward ? -1.0f : +1.0f;

    if (backward) {
        logger().info() << __FUNCTION__ << " BACKWARD : dist_brut=" << dist
                        << " ajustement=" << kBackwardPushAdjustmentMm
                        << " effective=" << effectiveDist << logs::end;
    }

    // Pose courante du robot avant la pousse - reference pour la visualisation
    // des rects (devant le robot dans le sens du cap, idx = config balise).
    const float initX = robot.asserv().pos_getX_mm();
    const float initY = robot.asserv().pos_getY_mm();
    const float initT = robot.asserv().pos_getTheta();

    // Sauvegarde du cap vitesse user (rest. apres manip pour ne pas laisser
    // 20% pour les tasks suivantes du JSON).
    const bool prevSpeedActive = robot.asserv().getUserMaxSpeedActive();
    const int  prevSpeedPct    = robot.asserv().getUserMaxSpeedPercent();
    auto restoreSpeed = [&] { robot.asserv().setMaxSpeed(prevSpeedActive, prevSpeedPct); };

    // Vitesse reduite pour la pousse (couple max, eviter de balayer les pieces).
    //robot.asserv().setMaxSpeed(true, 20);
    // Front center actif (collision sur element pousse), front lateral + back ignores.
    robot.actions().sensors().setIgnoreFrontNearObstacle(true, false, true);
    robot.actions().sensors().setIgnoreBackNearObstacle(true, false, true);

    Navigator nav(&robot, &robot.ia().iAbyPath());
    RetryPolicy policyPush = RetryPolicy::standard();   // 2 retries obstacle/collision

    // Visualisation SVG : 4 rects dans le sens de pousse a la position initiale.
    // L'idx dessine = pickupIdx initial (config balise physique), independant du
    // swap couleur applique pour la distance.
    drawConfigAtPose(robot, initX, initY, initT, pickupIdx, yellow,
                     /*extraForward=*/0.0f, /*dashed=*/false, /*backward=*/backward);

    TRAJ_STATE ts = nav.line(signDir * effectiveDist, policyPush);
    if (ts != TRAJ_FINISHED) {
        logger().error() << __FUNCTION__ << " " << zoneName << " avance ts=" << ts << logs::end;
        robot.asserv().resetEmergencyOnTraj();
        robot.svgPrintPosition();
        restoreSpeed();
        return false;
    }
    robot.svgPrintPosition();

    // Visualisation SVG : 4 rects a la position finale (fill semi-transparent
    // + stroke pointille). On utilise physicalPush = dist - kStartGapMm, soit
    // le deplacement REEL des cubes (le robot bouge effectiveDist mm, mais les
    // 85mm initiaux traversent le gap sans contact avec les cubes -> les cubes
    // ne sont pousses que des derniers (dist - kStartGapMm) mm).
    // En backward : dist - kStartGapMm = (effectiveDist - kBackwardPushAdjustmentMm) - kStartGapMm
    //             = effectiveDist - (kStartGapMm + kBackwardPushAdjustmentMm)
    //             = effectiveDist - 70 (= kCubeClosestFromCenterMm - kRobotRearOffset).
    // Meme valeur dans les deux modes : le deplacement physique des cubes
    // est independant de quelle face pousse.
    const float physicalPush = dist - kStartGapMm;
    drawConfigAtPose(robot, initX, initY, initT, pickupIdx, yellow,
                     /*extraForward=*/physicalPush, /*dashed=*/true, /*backward=*/backward);

    // Recul de degagement : direction opposee au sens de pousse.
    ts = nav.line(-signDir * D_RETREAT, policyPush);
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
// STUB - mouvement desactive tant que le corps de l'action n'est pas finalise.
// Plomberie adv position : OK depuis l'extension needed_adv_out_of_zone (Sensors
// publie adv_pos_centre_ via Asserv::setAdvPosCentre apres projection beacon->table).
// pos_getAdvPosition() retourne donc une vraie position quand la balise detecte
// (sinon (-100,-100) init = adv invalide, a traiter comme no-op).
// =============================================================================
bool defense_if_needed()
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    const bool yellow = robot.isMatchColor();

    // Coords BLEU (a calibrer/affiner demain si besoin)
    constexpr float ZONE1_XMIN_B = 1700.0f, ZONE1_XMAX_B = 2000.0f;
    constexpr float ZONE1_YMIN_B = 1300.0f, ZONE1_YMAX_B = 1800.0f;
    constexpr float DEF1_X_B = 1400.0f,     DEF1_Y_B = 1500.0f;

    constexpr float ZONE2_XMIN_B = 1700.0f, ZONE2_XMAX_B = 2000.0f;
    constexpr float ZONE2_YMIN_B =  700.0f, ZONE2_YMAX_B = 1100.0f;
    constexpr float DEF2_X_B = 1400.0f,     DEF2_Y_B =  900.0f;

    // (void) pour silence warnings tant que le corps est un stub
    (void) ZONE1_XMIN_B; (void) ZONE1_XMAX_B; (void) ZONE1_YMIN_B; (void) ZONE1_YMAX_B;
    (void) DEF1_X_B;     (void) DEF1_Y_B;
    (void) ZONE2_XMIN_B; (void) ZONE2_XMAX_B; (void) ZONE2_YMIN_B; (void) ZONE2_YMAX_B;
    (void) DEF2_X_B;     (void) DEF2_Y_B;

    // Position adv en repere TABLE (mm) - cf Asserv::pos_getAdvPosition().
    // Plomberie OK (cf Sensors::setAdvPosCentre apres projection beacon->table).
    // (-100,-100) = pas encore de detection valide -> stub log+no-op a la finalisation.
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
bool push_elements_zone(uint8_t pickupIdx, const char* zoneName, bool sensInverse, bool backward)
{
    return push_elements_zone_impl(pickupIdx, zoneName, sensInverse, backward);
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
    // En YELLOW, pickupForZone() lit la zone miroir physique (P1<->P11, etc.).
    registry.registerAction("push_elements_P1_B",
        [](){ return push_elements_zone(pickupForZone("P1"),  "P1",  true);  });
    registry.registerAction("push_elements_P1_H",
        [](){ return push_elements_zone(pickupForZone("P1"),  "P1",  false); });
    registry.registerAction("push_elements_P2_B",
        [](){ return push_elements_zone(pickupForZone("P2"),  "P2",  true);  });
    registry.registerAction("push_elements_P2_H",
        [](){ return push_elements_zone(pickupForZone("P2"),  "P2",  false); });
    registry.registerAction("push_elements_P11_B",
        [](){ return push_elements_zone(pickupForZone("P11"), "P11", true);  });
    registry.registerAction("push_elements_P11_H",
        [](){ return push_elements_zone(pickupForZone("P11"), "P11", false); });
    registry.registerAction("push_elements_P12_B",
        [](){ return push_elements_zone(pickupForZone("P12"), "P12", true);  });
    registry.registerAction("push_elements_P12_H",
        [](){ return push_elements_zone(pickupForZone("P12"), "P12", false); });

    // Horizontales (P3, P4, P13, P14) : _D = arrive de droite (INVERSE), _G = arrive de gauche (DIRECTE)
    registry.registerAction("push_elements_P3_D",
        [](){ return push_elements_zone(pickupForZone("P3"),  "P3",  true);  });
    registry.registerAction("push_elements_P3_G",
        [](){ return push_elements_zone(pickupForZone("P3"),  "P3",  false); });
    registry.registerAction("push_elements_P4_D",
        [](){ return push_elements_zone(pickupForZone("P4"),  "P4",  true);  });
    registry.registerAction("push_elements_P4_G",
        [](){ return push_elements_zone(pickupForZone("P4"),  "P4",  false); });
    registry.registerAction("push_elements_P13_D",
        [](){ return push_elements_zone(pickupForZone("P13"), "P13", true);  });
    registry.registerAction("push_elements_P13_G",
        [](){ return push_elements_zone(pickupForZone("P13"), "P13", false); });
    registry.registerAction("push_elements_P14_D",
        [](){ return push_elements_zone(pickupForZone("P14"), "P14", true);  });
    registry.registerAction("push_elements_P14_G",
        [](){ return push_elements_zone(pickupForZone("P14"), "P14", false); });

    // --- Push elements BACKWARD : 16 entrees (variantes rear-first des 16 ci-dessus) ---
    // Variante rear-first : nav.line(-dist) au lieu de nav.line(+dist), face
    // arriere a 130mm du centre (vs 115mm avant), ajustement geometrique -15mm
    // (= kRobotFrontOffset - kRobotRearOffset) sur la distance brute. Le
    // faceTo initial du robot reste a la charge de la strategie (idem forward).
    // Cf robot/md/PUSH_ELEMENTS_2026.md.
    //
    // Memes conventions de suffixe que forward (B/H pour verticales,
    // D/G pour horizontales).

    // Verticales backward (P1, P2, P11, P12) - meme regle de mapping pickup que forward.
    registry.registerAction("push_back_elements_P1_B",
        [](){ return push_elements_zone(pickupForZone("P1"),  "P1",  true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P1_H",
        [](){ return push_elements_zone(pickupForZone("P1"),  "P1",  false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P2_B",
        [](){ return push_elements_zone(pickupForZone("P2"),  "P2",  true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P2_H",
        [](){ return push_elements_zone(pickupForZone("P2"),  "P2",  false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P11_B",
        [](){ return push_elements_zone(pickupForZone("P11"), "P11", true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P11_H",
        [](){ return push_elements_zone(pickupForZone("P11"), "P11", false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P12_B",
        [](){ return push_elements_zone(pickupForZone("P12"), "P12", true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P12_H",
        [](){ return push_elements_zone(pickupForZone("P12"), "P12", false, /*backward=*/true); });

    // Horizontales backward (P3, P4, P13, P14)
    registry.registerAction("push_back_elements_P3_D",
        [](){ return push_elements_zone(pickupForZone("P3"),  "P3",  true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P3_G",
        [](){ return push_elements_zone(pickupForZone("P3"),  "P3",  false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P4_D",
        [](){ return push_elements_zone(pickupForZone("P4"),  "P4",  true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P4_G",
        [](){ return push_elements_zone(pickupForZone("P4"),  "P4",  false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P13_D",
        [](){ return push_elements_zone(pickupForZone("P13"), "P13", true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P13_G",
        [](){ return push_elements_zone(pickupForZone("P13"), "P13", false, /*backward=*/true); });
    registry.registerAction("push_back_elements_P14_D",
        [](){ return push_elements_zone(pickupForZone("P14"), "P14", true,  /*backward=*/true); });
    registry.registerAction("push_back_elements_P14_G",
        [](){ return push_elements_zone(pickupForZone("P14"), "P14", false, /*backward=*/true); });


}
