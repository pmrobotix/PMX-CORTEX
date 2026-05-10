/*!
 * \file O_PushElementsTest.cpp
 * \brief Test du module push_elements (validation logique + poussage reel).
 */

#include "O_PushElementsTest.hpp"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "OPOS6UL_RobotExtended.hpp"
#include "OPOS6UL_AsservExtended.hpp"
#include "OPOS6UL_ActionsExtended.hpp"
#include "StrategyActions2026.hpp"
#include "log/SvgWriter.hpp"
#include "utils/Arguments.hpp"

O_PushElementsTest::O_PushElementsTest()
    : FunctionalTest("PushElements", "Validation push_elements (logique + poussage reel)", "pe")
{
}

namespace {

// Helper pour accumuler PASS/FAIL avec affichage tabulaire.
struct TestStats
{
    int total = 0;
    int passed = 0;

    void check(const char* label, float computed, float expected)
    {
        ++total;
        const bool ok = std::fabs(computed - expected) < 0.01f;
        if (ok) ++passed;
        std::cout << "  " << std::setw(3) << total << ". "
                  << std::left << std::setw(60) << label << std::right
                  << " computed=" << std::setw(7) << std::fixed << std::setprecision(1) << computed
                  << " expected=" << std::setw(7) << std::fixed << std::setprecision(1) << expected
                  << "  " << (ok ? "OK" : "FAIL") << std::endl;
    }
};

// Construit un label lisible pour le cas teste.
std::string label(const char* zone, bool sensInverse, bool yellow, int idx)
{
    std::ostringstream oss;
    oss << zone << " " << (sensInverse ? "INV" : "DIR")
        << " idx=" << idx
        << " "    << (yellow ? "YELLOW" : "BLUE");
    return oss.str();
}

// Convertit le suffixe (B/H/D/G) en sensInverse.
//   _B (arrive bas)    -> sens INVERSE
//   _H (arrive haut)   -> sens DIRECTE
//   _D (arrive droite) -> sens INVERSE
//   _G (arrive gauche) -> sens DIRECTE
bool suffixeToInverse(const char* suffixe, bool& outOk)
{
    outOk = true;
    if (suffixe == nullptr || suffixe[0] == '\0' || suffixe[1] != '\0') {
        outOk = false;
        return false;
    }
    switch (suffixe[0]) {
        case 'B': case 'b': return true;
        case 'H': case 'h': return false;
        case 'D': case 'd': return true;
        case 'G': case 'g': return false;
        default: outOk = false; return false;
    }
}

// Force la valeur pickup_P{N} sur le robot pour le mode 2. Retourne false si
// zone non reconnue ou phase >= PHASE_MATCH (setPickup* refuse).
bool setPickupForZone(OPOS6UL_RobotExtended& robot, const char* zone, uint8_t idx)
{
    if (idx > 5) return false;
    if (zone == nullptr) return false;
    if      (std::strcmp(zone, "P1")  == 0) return robot.setPickupP1(idx);
    else if (std::strcmp(zone, "P2")  == 0) return robot.setPickupP2(idx);
    else if (std::strcmp(zone, "P3")  == 0) return robot.setPickupP3(idx);
    else if (std::strcmp(zone, "P4")  == 0) return robot.setPickupP4(idx);
    else if (std::strcmp(zone, "P11") == 0) return robot.setPickupP11(idx);
    else if (std::strcmp(zone, "P12") == 0) return robot.setPickupP12(idx);
    else if (std::strcmp(zone, "P13") == 0) return robot.setPickupP13(idx);
    else if (std::strcmp(zone, "P14") == 0) return robot.setPickupP14(idx);
    return false;
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
//
// Ainsi `pe P1 B 0` (idx=0 BBYY, cap=+90 = vient du bas) affiche YYBB du proche
// au loin (le robot voit les Y devant lui, les B au fond), et utilise la table
// distInverse (sensInverse=true). En `pe P1 H 0`, le robot voit BBYY du proche
// au loin et utilise distDirecte (sensInverse=false). Identique au match.

// Dimensions d'un rectangle (mm).
//   short = profondeur dans le sens de pousse (forward)
//   long  = largeur perpendiculaire
constexpr float kRectLong         = 150.0f;
constexpr float kRectShort        =  50.0f;
// Distance du centre robot a la face octogonale avant (mm).
constexpr float kRobotFrontOffset = 115.0f;

// Sequence des 4 couleurs par idx balise.
//   0=BBYY 1=YYBB 2=BYYB 3=YBBY 4=BYBY 5=YBYB
constexpr const char* kIdxToSequence[6] = {
    "BBYY", "YYBB", "BYYB", "YBBY", "BYBY", "YBYB"
};

// Retourne true si char[0] doit etre cote robot (le plus proche).
//   Vertical-dominant (|sin| > |cos|) : sin < 0 => cap Y-, dans sens lecture.
//   Horizontal-dominant                : cos > 0 => cap X+, dans sens lecture.
bool char0IsClosestToRobot(float theta_rad)
{
    const float c = std::cos(theta_rad);
    const float s = std::sin(theta_rad);
    if (std::fabs(s) > std::fabs(c)) {
        return s < 0.0f;          // vertical : lecture Y-
    }
    return c > 0.0f;              // horizontal : lecture X+
}

// Dessine la config (4 rectangles) devant le robot, a la pose donnee, decalee
// en avant de `extraForward` mm (0 pour position initiale, `dist` pour finale).
// dashed=false : fill plein. dashed=true : pointilles + transparence.
void drawConfigAtPose(OPOS6UL_RobotExtended& robot,
                      float poseX, float poseY, float poseTheta_rad,
                      uint8_t idx, bool /*yellow*/,
                      float extraForward, bool dashed)
{
    if (idx > 5) return;

    // L'idx vient de la balise = config physique fixe (char[0] au HAUT/GAUCHE,
    // char[3] au BAS/DROITE). Pas de swap d'affichage selon couleur ou cap.
    const char* seq = kIdxToSequence[idx];

    // Vecteurs unitaires : forward (sens cap), left (perpendiculaire +90 deg).
    const float fx = std::cos(poseTheta_rad);
    const float fy = std::sin(poseTheta_rad);
    const float lx = -fy;
    const float ly =  fx;

    // Demi-dimensions
    const float hF = kRectShort / 2.0f;   // 25mm dans le sens forward
    const float hL = kRectLong  / 2.0f;   // 75mm dans le sens perpendiculaire

    const bool char0Closest = char0IsClosestToRobot(poseTheta_rad);

    for (int i = 0; i < 4; ++i) {
        // i = position dans la pile depuis le robot (0 = proche, 3 = loin).
        // L'indice de caractere dans la sequence depend du sens de lecture.
        const int  charIdx = char0Closest ? i : (3 - i);
        const char c       = seq[charIdx];

        // Distance forward du centre du rect[i] depuis le centre robot.
        const float fwd = kRobotFrontOffset + extraForward
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

} // namespace

void O_PushElementsTest::runValidation()
{
    using namespace push_elements_test_api;

    std::cout << "\n=== O_PushElementsTest : validation logique ===" << std::endl;
    std::cout << "Verifie la mecanique de combinaison (mapping idx, swap couleur,"
              << " flip suffixe horizontal, offsets). Les attendus sont derives"
              << " de D_BASE + dist[idx] (+offset) -> reste vert si on retouche"
              << " la calibration." << std::endl;
    std::cout << std::endl;

    const float B = dBaseMm();  // raccourci pour la lisibilite des attendus.
    TestStats stats;

    // -------------------------------------------------------------------------
    // Groupe 1 : BLEU passthrough (zone verticale P1, pas d'offset, pas de swap).
    // -------------------------------------------------------------------------
    std::cout << "[Groupe 1] BLEU passthrough P1 (verticale, sans offset)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = B + distDirecteAt(k);
        stats.check(label("P1", false, false, k).c_str(),
                    computeDistance(k, "P1", false, false), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = B + distInverseAt(k);
        stats.check(label("P1", true, false, k).c_str(),
                    computeDistance(k, "P1", true, false), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 2 : YELLOW swap idx sur P1 (verticale -> pas de flip suffixe).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 2] YELLOW swap idx sur P1 (verticale, pas de flip suffixe)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = B + distDirecteAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P1", false, true, k).c_str(),
                    computeDistance(k, "P1", false, true), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = B + distInverseAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P1", true, true, k).c_str(),
                    computeDistance(k, "P1", true, true), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 3 : YELLOW + zone HORIZONTALE -> flip suffixe + swap idx.
    // P3 sans offset. Input sensInverse=false -> apres flip = true -> distInverse.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 3] YELLOW horizontale P3 (flip suffixe + swap idx)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        // Input directe -> flip horiz -> inverse -> B + distInverseAt(SWAP[k])
        const float exp = B + distInverseAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P3", false, true, k).c_str(),
                    computeDistance(k, "P3", false, true), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        // Input inverse -> flip horiz -> directe -> B + distDirecteAt(SWAP[k])
        const float exp = B + distDirecteAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P3", true, true, k).c_str(),
                    computeDistance(k, "P3", true, true), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 4 : Offsets P4 et P14.
    // P4 BLEU directe : B + distDirecteAt(k) + zoneOffset_P4.
    // P14 YELLOW directe : flip horiz -> inverse -> B + distInverseAt(SWAP[k]) + offset_P14.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 4] Offsets P4 / P14" << std::endl;
    {
        const uint8_t k = 0;
        const float expP4 = B + distDirecteAt(k) + zoneOffsetFor("P4");
        stats.check(label("P4", false, false, k).c_str(),
                    computeDistance(k, "P4", false, false), expP4);
    }
    {
        const uint8_t k = 2;  // BYYB (table la plus longue) pour eviter dist<=0
        const float expP4 = B + distInverseAt(k) + zoneOffsetFor("P4");
        stats.check(label("P4", true, false, k).c_str(),
                    computeDistance(k, "P4", true, false), expP4);
    }
    {
        const uint8_t k = 0;
        const float expP14 = B + distInverseAt(SWAP_COLOR_IDX[k]) + zoneOffsetFor("P14");
        stats.check(label("P14", false, true, k).c_str(),
                    computeDistance(k, "P14", false, true), expP14);
    }
    {
        const uint8_t k = 2;
        const float expP14 = B + distDirecteAt(SWAP_COLOR_IDX[k]) + zoneOffsetFor("P14");
        stats.check(label("P14", true, true, k).c_str(),
                    computeDistance(k, "P14", true, true), expP14);
    }

    // -------------------------------------------------------------------------
    // Groupe 5 : garde-fous (idx invalide -> -1).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 5] Garde-fous" << std::endl;
    stats.check("idx=99 P1 BLUE -> erreur (-1)",
                computeDistance(99, "P1", false, false), -1.0f);
    stats.check("idx=6 P1 BLUE -> erreur (-1)",
                computeDistance(6, "P1", false, false), -1.0f);

    // -------------------------------------------------------------------------
    // Recap.
    // -------------------------------------------------------------------------
    std::cout << "\n=== Bilan : " << stats.passed << "/" << stats.total
              << " " << (stats.passed == stats.total ? "PASS" : "FAIL")
              << " ===\n" << std::endl;
    logger().info() << "validation logique : " << stats.passed << "/" << stats.total
                    << (stats.passed == stats.total ? " PASS" : " FAIL") << logs::end;
}

void O_PushElementsTest::runPushReel(const char* zone, const char* suffixe, int idx)
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();

    bool ok = false;
    const bool sensInverse = suffixeToInverse(suffixe, ok);
    if (!ok) {
        logger().error() << "suffixe '" << (suffixe ? suffixe : "(null)")
                         << "' invalide (attendu B/H/D/G)" << logs::end;
        return;
    }
    if (idx < 0 || idx > 5) {
        logger().error() << "idx=" << idx << " hors plage (attendu 0..5)" << logs::end;
        return;
    }

    // Position initiale via /+ coordx coordy coorda (defaut 200 600 90 = entree
    // P1_B selon strategyPMX2). Permet de placer le robot avant la pousse pour
    // visualiser le resultat dans le SVG (calibration des distances D1..D4 /
    // decal).
    Arguments args = robot.getArgs();
    const float coordx     = std::atof(args['+']["coordx"].c_str());
    const float coordy     = std::atof(args['+']["coordy"].c_str());
    const float coorda_deg = std::atof(args['+']["coorda"].c_str());
    logger().info() << "Position initiale x=" << coordx << " y=" << coordy
                    << " a=" << coorda_deg << logs::end;

    robot.asserv().setPositionAndColor(coordx, coordy, coorda_deg, robot.isMatchColor());
    robot.asserv().startMotionTimerAndOdo(false);
    robot.asserv().assistedHandling();
    robot.svgPrintPosition();
    robot.actions().start();

    // Force la valeur pickup_P{N} : utile en simu sans balise. La phase doit
    // etre < PHASE_MATCH (donc le test doit tourner avant tirette/start).
    if (!setPickupForZone(robot, zone, static_cast<uint8_t>(idx))) {
        logger().warn() << "setPickupForZone failed (zone=" << zone << " idx=" << idx
                        << ", phase peut-etre >= PHASE_MATCH ou zone inconnue)"
                        << " -> push utilisera la valeur courante" << logs::end;
    }

    // Pose courante du robot (apres setPositionAndColor) - reference pour la
    // visualisation des rects (devant le robot dans le sens du cap).
    const float initX = robot.asserv().pos_getX_mm();
    const float initY = robot.asserv().pos_getY_mm();
    const float initT = robot.asserv().pos_getTheta();
    const bool  yellow = robot.isMatchColor();

    // Visualisation SVG : 4 rects devant le robot a la position initiale.
    drawConfigAtPose(robot, initX, initY, initT, static_cast<uint8_t>(idx), yellow,
                     /*extraForward=*/0.0f, /*dashed=*/false);

    logger().info() << "[mode poussage reel] push_elements_" << zone << "_" << suffixe
                    << " idx=" << idx
                    << " sens=" << (sensInverse ? "INVERSE" : "DIRECTE")
                    << " color=" << (yellow ? "YELLOW" : "BLUE")
                    << logs::end;

    const bool result = push_elements_zone(static_cast<uint8_t>(idx), zone, sensInverse);
    logger().info() << "push_elements_zone result=" << (result ? "true" : "false") << logs::end;
    robot.svgPrintPosition();

    // Visualisation SVG : 4 rects a la position finale = pose initiale + dist
    // dans le sens du cap initial (fill semi-transparent + stroke pointille).
    const float dist = push_elements_test_api::computeDistance(
        static_cast<uint8_t>(idx), zone, sensInverse, yellow);
    if (dist > 0.0f) {
        drawConfigAtPose(robot, initX, initY, initT, static_cast<uint8_t>(idx), yellow,
                         /*extraForward=*/dist, /*dashed=*/true);
    }
}

void O_PushElementsTest::configureConsoleArgs(int argc, char** argv)
{
    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    robot.getArgs().addArgument("zone", "zone P1..P14 (none -> mode validation)", "none");
    robot.getArgs().addArgument("suffixe", "suffixe B|H|D|G", "none");
    robot.getArgs().addArgument("idx", "idx 0..5", "-1");

    Arguments::Option cOptMultiplier('M', "simu speed multiplier (0=instantane, 1.0=temps reel)");
    cOptMultiplier.addArgument("multiplier", "multiplier", "1.0");
    robot.getArgs().addOption(cOptMultiplier);

    // Position initiale (defauts = entree zone P1_B selon strategyPMX2 :
    // x=200 y=600 cap Y+, robot arrive du bas).
    Arguments::Option cOptPos('+', "Coordinates x,y,a (position initiale avant push)");
    cOptPos.addArgument("coordx", "coord x mm",      "200.0");
    cOptPos.addArgument("coordy", "coord y mm",      "600.0");
    cOptPos.addArgument("coorda", "coord teta deg",   "90.0");
    robot.getArgs().addOption(cOptPos);

    robot.parseConsoleArgs(argc, argv);
}

void O_PushElementsTest::run(int argc, char** argv)
{
    logger().info() << "N° " << position() << " - Executing - " << desc() << logs::end;
    configureConsoleArgs(argc, argv);

    OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
    Arguments args = robot.getArgs();

    const float multiplier = std::atof(args['M']["multiplier"].c_str());
    robot.asserv().setSimuSpeedMultiplier(multiplier);
    logger().info() << "simuSpeed=" << multiplier << logs::end;

    const std::string zone    = args["zone"];
    const std::string suffixe = args["suffixe"];
    const int         idx     = std::atoi(args["idx"].c_str());

    if (zone.empty() || zone == "none") {
        runValidation();
        return;
    }
    if (suffixe.empty() || suffixe == "none" || idx < 0) {
        logger().error() << "Mode poussage reel : <zone> <suffixe> <idx> tous obligatoires."
                         << " zone='" << zone << "' suffixe='" << suffixe
                         << "' idx=" << idx << logs::end;
        std::cout << usageHelp() << std::endl;
        return;
    }
    runPushReel(zone.c_str(), suffixe.c_str(), idx);
}
