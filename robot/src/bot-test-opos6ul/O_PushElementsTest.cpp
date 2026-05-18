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

// Replique le mapping miroir physique P{N}<->P{N+10} (cf mirrorZone() dans
// StrategyActions2026.cpp, non expose hors du fichier). Retourne zone inchangee
// si aucun mapping.
const char* mirrorZoneTest(const char* zone)
{
    if (zone == nullptr) return zone;
    if (std::strcmp(zone, "P1")  == 0) return "P11";
    if (std::strcmp(zone, "P2")  == 0) return "P12";
    if (std::strcmp(zone, "P3")  == 0) return "P13";
    if (std::strcmp(zone, "P4")  == 0) return "P14";
    if (std::strcmp(zone, "P11") == 0) return "P1";
    if (std::strcmp(zone, "P12") == 0) return "P2";
    if (std::strcmp(zone, "P13") == 0) return "P3";
    if (std::strcmp(zone, "P14") == 0) return "P4";
    return zone;
}

// Calcule l'attendu de computeDistance() en repliquant la logique :
// swap idx + flip suffixe horizontal en YELLOW, override par zone physique
// (sur idx post-swap), puis D_BASE + dist[idx] + zoneOffset.
float expectedDistance(uint8_t idx, const char* zone, bool sensInverse, bool yellow)
{
    using namespace push_elements_test_api;
    uint8_t     postIdx  = idx;
    bool        inv      = sensInverse;
    if (yellow) {
        if (isHorizontalZone(zone)) inv = !inv;
        postIdx = SWAP_COLOR_IDX[idx];
    }
    const char* physZone = yellow ? mirrorZoneTest(zone) : zone;
    const float ov       = distOverrideFor(physZone, postIdx);
    // kNoOverride = 1e9f cote StrategyActions2026.cpp ; on teste la sentinelle
    // avec un seuil large (toute distance reelle est tres inferieure a 1e8).
    const bool  hasOv    = (ov < 1.0e8f);
    const float distBase = hasOv ? ov
                                 : (inv ? distInverseAt(postIdx) : distDirecteAt(postIdx));
    return dBaseMm() + distBase + zoneOffsetFor(zone);
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
    // En YELLOW, physZone = mirrorZone("P3") = "P13" : l'override -175 de P13
    // s'applique a P3 YELLOW idx -> SWAP[idx]==2. expectedDistance() en tient
    // compte (les attendus restent verts apres l'ajout de l'override).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 3] YELLOW horizontale P3 (flip suffixe + swap idx)" << std::endl;
    for (uint8_t k = 0; k < 6; ++k) {
        // Input directe -> flip horiz -> inverse ; override sur physZone "P13".
        stats.check(label("P3", false, true, k).c_str(),
                    computeDistance(k, "P3", false, true),
                    expectedDistance(k, "P3", false, true));
    }
    for (uint8_t k = 0; k < 6; ++k) {
        // Input inverse -> flip horiz -> directe ; override sur physZone "P13".
        stats.check(label("P3", true, true, k).c_str(),
                    computeDistance(k, "P3", true, true),
                    expectedDistance(k, "P3", true, true));
    }

    // -------------------------------------------------------------------------
    // Groupe 4 : Offsets P4 et P14 + override de dist[idx].
    // Les attendus sont derives par expectedDistance() qui replique la logique
    // de computeDistance (swap idx, flip suffixe horizontal, override par zone
    // physique, offset). P4/P14 ont un override -175 @ idx 2 (post-swap).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 4] Offsets P4 / P14 + override dist[idx]" << std::endl;
    {
        // k=0 : pas d'override (kNoOverride @ idx 0) -> table partagee.
        const uint8_t k = 0;
        stats.check(label("P4", false, false, k).c_str(),
                    computeDistance(k, "P4", false, false),
                    expectedDistance(k, "P4", false, false));
    }
    {
        // k=2 : OVERRIDE -175 actif (P4 idx 2). distInverse[2] ignoree.
        const uint8_t k = 2;
        stats.check(label("P4", true, false, k).c_str(),
                    computeDistance(k, "P4", true, false),
                    expectedDistance(k, "P4", true, false));
    }
    {
        // P14 YELLOW : physZone = mirrorZone("P14") = "P4". k=0 -> pas d'override.
        const uint8_t k = 0;
        stats.check(label("P14", false, true, k).c_str(),
                    computeDistance(k, "P14", false, true),
                    expectedDistance(k, "P14", false, true));
    }
    {
        // P14 YELLOW : SWAP[k=3]=2 -> physZone "P4" idx 2 -> OVERRIDE -175 actif.
        const uint8_t k = 3;
        stats.check(label("P14", true, true, k).c_str(),
                    computeDistance(k, "P14", true, true),
                    expectedDistance(k, "P14", true, true));
    }

    // -------------------------------------------------------------------------
    // Groupe 4b : override de zone "P inversee" P13 + miroir YELLOW.
    // P13 a un override -175 @ idx 2. Verifie en particulier le comportement
    // yellow : wrapper "P13" en yellow -> physiquement P3 -> PAS d'override ;
    // wrapper "P3" en yellow -> physiquement P13 -> override -175.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 4b] Override P13 + miroir YELLOW (P3<->P13)" << std::endl;
    {
        // P13 BLEU directe, idx 2 -> override -175 (zone physique P13).
        const uint8_t k = 2;
        stats.check(label("P13", false, false, k).c_str(),
                    computeDistance(k, "P13", false, false),
                    expectedDistance(k, "P13", false, false));
    }
    {
        // P13 BLEU directe, idx 0 -> pas d'override -> table partagee.
        const uint8_t k = 0;
        stats.check(label("P13", false, false, k).c_str(),
                    computeDistance(k, "P13", false, false),
                    expectedDistance(k, "P13", false, false));
    }
    {
        // P13 YELLOW : physZone = mirrorZone("P13") = "P3" -> PAS d'override
        // meme sur l'idx post-swap. SWAP[k=3]=2 mais P3 n'est pas dans la table.
        const uint8_t k = 3;
        stats.check(label("P13", false, true, k).c_str(),
                    computeDistance(k, "P13", false, true),
                    expectedDistance(k, "P13", false, true));
    }
    {
        // P3 YELLOW : physZone = mirrorZone("P3") = "P13". SWAP[k=3]=2 ->
        // P13 idx 2 -> OVERRIDE -175 actif (miroir yellow).
        const uint8_t k = 3;
        stats.check(label("P3", false, true, k).c_str(),
                    computeDistance(k, "P3", false, true),
                    expectedDistance(k, "P3", false, true));
    }
    {
        // P3 BLEU : physZone = "P3" -> jamais d'override (P3 absent de la table).
        const uint8_t k = 2;
        stats.check(label("P3", false, false, k).c_str(),
                    computeDistance(k, "P3", false, false),
                    expectedDistance(k, "P3", false, false));
    }

    // -------------------------------------------------------------------------
    // Groupe 5 : mapping P{N} <-> P{N+10} en YELLOW (3eme transformation couleur).
    // En YELLOW le miroir Asserv place le robot sur la zone miroir physique :
    // push_elements_P1_* doit lire pickupP11() (pas pickupP1()), etc.
    // resolvePickupForZone est appele par les wrappers via pickupForZone() en
    // utilisant isMatchColor() du robot. Ici on injecte le booleen pour tester
    // les 2 couleurs sans toucher au robot.
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 5] Mapping P{N}<->P{N+10} en YELLOW (resolvePickupForZone)"
              << std::endl;
    {
        OPOS6UL_RobotExtended& robot = OPOS6UL_RobotExtended::instance();
        struct Pair { const char* low; const char* high; uint8_t vLow; uint8_t vHigh; };
        const Pair pairs[] = {
            { "P1",  "P11", 2, 4 },
            { "P2",  "P12", 1, 3 },
            { "P3",  "P13", 0, 5 },
            { "P4",  "P14", 3, 2 },
        };
        for (const auto& p : pairs) {
            // Set 2 valeurs differentes pour les 2 zones du couple.
            const bool okLow  = setPickupForZone(robot, p.low,  p.vLow);
            const bool okHigh = setPickupForZone(robot, p.high, p.vHigh);
            if (!okLow || !okHigh) {
                // Phase >= MATCH ou autre refus : on cast a -1 pour visualiser
                // l'echec dans le tableau au lieu de skipper silencieusement.
                stats.check("setPickupForZone failed (phase>=MATCH ?)",
                            -1.0f, 0.0f);
                continue;
            }

            // BLEU : passthrough -> resolve(low)=vLow, resolve(high)=vHigh.
            {
                std::ostringstream lbl;
                lbl << p.low  << " BLUE   -> pickup" << p.low;
                stats.check(lbl.str().c_str(),
                            static_cast<float>(resolvePickupForZone(p.low, false)),
                            static_cast<float>(p.vLow));
            }
            {
                std::ostringstream lbl;
                lbl << p.high << " BLUE   -> pickup" << p.high;
                stats.check(lbl.str().c_str(),
                            static_cast<float>(resolvePickupForZone(p.high, false)),
                            static_cast<float>(p.vHigh));
            }
            // YELLOW : swap zone miroir -> resolve(low)=vHigh, resolve(high)=vLow.
            {
                std::ostringstream lbl;
                lbl << p.low  << " YELLOW -> pickup" << p.high << " (swap miroir)";
                stats.check(lbl.str().c_str(),
                            static_cast<float>(resolvePickupForZone(p.low, true)),
                            static_cast<float>(p.vHigh));
            }
            {
                std::ostringstream lbl;
                lbl << p.high << " YELLOW -> pickup" << p.low  << " (swap miroir)";
                stats.check(lbl.str().c_str(),
                            static_cast<float>(resolvePickupForZone(p.high, true)),
                            static_cast<float>(p.vLow));
            }
        }
    }

    // -------------------------------------------------------------------------
    // Groupe 6 : garde-fous (idx invalide -> -1).
    // -------------------------------------------------------------------------
    std::cout << "\n[Groupe 6] Garde-fous" << std::endl;
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

    // La visualisation SVG des 4 rects (avant/apres pousse) est faite en interne
    // par push_elements_zone() (dans StrategyActions2026.cpp), pour qu'elle fire
    // aussi quand la manip est invoquee depuis une strategie JSON.
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
