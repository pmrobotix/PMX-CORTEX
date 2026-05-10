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

} // namespace

void O_PushElementsTest::runValidation()
{
    using namespace push_elements_test_api;

    std::cout << "\n=== O_PushElementsTest : validation logique ===" << std::endl;
    std::cout << "Verifie la mecanique de combinaison (mapping idx, swap couleur,"
              << " flip suffixe horizontal, offsets). Les attendus sont derives"
              << " des constantes -> reste vert si on retouche D1..D4." << std::endl;
    std::cout << std::endl;

    TestStats stats;

    // -------------------------------------------------------------------------
    // Groupe 1 : BLEU passthrough (zone verticale P1, pas d'offset, pas de swap).
    // -------------------------------------------------------------------------
    std::cout << "[Groupe 1] BLEU passthrough P1 (verticale, sans offset)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = distDirecteAt(k);
        stats.check(label("P1", false, false, k).c_str(),
                    computeDistance(k, "P1", false, false), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = distInverseAt(k);
        stats.check(label("P1", true, false, k).c_str(),
                    computeDistance(k, "P1", true, false), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 2 : YELLOW swap idx sur P1 (verticale -> pas de flip suffixe).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 2] YELLOW swap idx sur P1 (verticale, pas de flip suffixe)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = distDirecteAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P1", false, true, k).c_str(),
                    computeDistance(k, "P1", false, true), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        const float exp = distInverseAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P1", true, true, k).c_str(),
                    computeDistance(k, "P1", true, true), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 3 : YELLOW + zone HORIZONTALE -> flip suffixe + swap idx.
    // P3 sans offset. Input sensInverse=false -> apres flip = true -> distInverse.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 3] YELLOW horizontale P3 (flip suffixe + swap idx)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        // Input directe -> flip horiz -> inverse -> distInverseAt(SWAP[k])
        const float exp = distInverseAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P3", false, true, k).c_str(),
                    computeDistance(k, "P3", false, true), exp);
    }
    for (uint8_t k = 0; k < 6; ++k) {
        // Input inverse -> flip horiz -> directe -> distDirecteAt(SWAP[k])
        const float exp = distDirecteAt(SWAP_COLOR_IDX[k]);
        stats.check(label("P3", true, true, k).c_str(),
                    computeDistance(k, "P3", true, true), exp);
    }

    // -------------------------------------------------------------------------
    // Groupe 4 : Offsets P4 et P14.
    // P4 BLEU directe : distDirecteAt(k) + zoneOffset_P4.
    // P14 YELLOW directe : flip horiz -> inverse -> distInverseAt(SWAP[k]) + offset_P14.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 4] Offsets P4 / P14" << std::endl;
    {
        const uint8_t k = 0;
        const float expP4 = distDirecteAt(k) + zoneOffsetFor("P4");
        stats.check(label("P4", false, false, k).c_str(),
                    computeDistance(k, "P4", false, false), expP4);
    }
    {
        const uint8_t k = 2;  // BYYB -> D2 (la plus longue) pour eviter dist<=0
        const float expP4 = distInverseAt(k) + zoneOffsetFor("P4");
        stats.check(label("P4", true, false, k).c_str(),
                    computeDistance(k, "P4", true, false), expP4);
    }
    {
        const uint8_t k = 0;
        const float expP14 = distInverseAt(SWAP_COLOR_IDX[k]) + zoneOffsetFor("P14");
        stats.check(label("P14", false, true, k).c_str(),
                    computeDistance(k, "P14", false, true), expP14);
    }
    {
        const uint8_t k = 2;
        const float expP14 = distDirecteAt(SWAP_COLOR_IDX[k]) + zoneOffsetFor("P14");
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

    logger().info() << "[mode poussage reel] push_elements_" << zone << "_" << suffixe
                    << " idx=" << idx
                    << " sens=" << (sensInverse ? "INVERSE" : "DIRECTE")
                    << " color=" << (robot.isMatchColor() ? "YELLOW" : "BLUE")
                    << logs::end;

    const bool result = push_elements_zone(static_cast<uint8_t>(idx), zone, sensInverse);
    logger().info() << "push_elements_zone result=" << (result ? "true" : "false") << logs::end;
    robot.svgPrintPosition();
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
