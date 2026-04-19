/*!
 * \file
 * \brief Implementation du test unitaire IsOnPath.
 *
 * Tests TDD (ecrits avant implementation) du predicat ObstacleZone::isOnPath.
 * Toutes les positions sont des CENTRES (robot, cible, adversaire).
 *
 * Sorties de validation :
 *   - Log ASCII dans la console pour chaque cas (robot/cible/adv, attendu/obtenu).
 *   - Fichier SVG "isonpath_tests.svg" ecrit A COTE DU BINAIRE de test
 *     avec les 10 scenes en grille 2x5 (ouvrir dans un navigateur).
 *
 * Parametres de reference (PMX 2026) :
 *   robot diametre    = 280 mm (= 140 mm rayon)
 *   adv diametre      = 400 mm (= 200 mm rayon, inclut marge)
 *   corridor_width_mm = 280 + 400 = 680 mm (demi = 340 mm)
 *   stop_distance_mm  = 460 mm (seuil BLOCKING depuis centre robot)
 *   slow_distance_mm  = 620 mm (seuil APPROACHING depuis centre robot)
 */

#include "IsOnPathTest.hpp"

#include "../../src/common/geometry/ObstacleZone.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

// Parametres de reference des tests
constexpr float CORRIDOR_WIDTH = 680.0f;
constexpr float SLOW_DIST      = 620.0f;
constexpr float STOP_DIST      = 460.0f;
constexpr float ROBOT_RADIUS   = 140.0f;
constexpr float ADV_RADIUS     = 200.0f;

struct Scene {
	std::string  name;
	float        x_robot, y_robot;
	float        x_target, y_target;
	float        x_adv, y_adv;
	PathStatus   expected;
	PathStatus   actual;
};

// Accumulateur global des scenes pour generation du SVG en fin de suite
std::vector<Scene> g_scenes;

const char* toStr(PathStatus s)
{
	switch (s) {
		case PathStatus::CLEAR:       return "CLEAR";
		case PathStatus::APPROACHING: return "APPROACHING";
		case PathStatus::BLOCKING:    return "BLOCKING";
	}
	return "?";
}

const char* svgFill(PathStatus s)
{
	switch (s) {
		case PathStatus::CLEAR:       return "#9ACD32";  // yellow-green
		case PathStatus::APPROACHING: return "#FFA500";  // orange
		case PathStatus::BLOCKING:    return "#DC143C";  // crimson
	}
	return "gray";
}

// Repertoire du binaire : /proc/self/exe -> dirname
std::string exeDirectory()
{
	char buf[PATH_MAX];
	ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
	if (n <= 0) return ".";
	buf[n] = '\0';
	std::string path(buf);
	size_t slash = path.find_last_of('/');
	return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

// Helper : lance isOnPath, stocke la scene, retourne le statut
PathStatus runScene(const std::string& name,
                    float x_r, float y_r,
                    float x_t, float y_t,
                    float x_a, float y_a,
                    PathStatus expected)
{
	ObstacleZone oz;
	PathStatus actual = oz.isOnPath(
			x_r, y_r, x_t, y_t, x_a, y_a,
			CORRIDOR_WIDTH, SLOW_DIST, STOP_DIST);

	Scene sc{name, x_r, y_r, x_t, y_t, x_a, y_a, expected, actual};
	g_scenes.push_back(sc);

	return actual;
}

// Helper : log ASCII d'une scene
void logScene(const Scene& sc)
{
	const logs::Logger& log = logs::LoggerFactory::logger("IsOnPathTest");

	float dx = sc.x_target - sc.x_robot;
	float dy = sc.y_target - sc.y_robot;
	float len = std::sqrt(dx*dx + dy*dy);
	float along = 0.0f, lateral = 0.0f;
	if (len > 0.001f) {
		float t = ((sc.x_adv - sc.x_robot) * dx + (sc.y_adv - sc.y_robot) * dy) / (len*len);
		float px = sc.x_robot + t * dx;
		float py = sc.y_robot + t * dy;
		along   = t * len;
		lateral = std::sqrt((sc.x_adv - px)*(sc.x_adv - px) + (sc.y_adv - py)*(sc.y_adv - py));
	}

	const char* verdict = (sc.actual == sc.expected) ? "PASS" : "FAIL";
	log.info() << sc.name
	           << "  robot(" << sc.x_robot << "," << sc.y_robot << ")"
	           << "  cible(" << sc.x_target << "," << sc.y_target << ")"
	           << "  adv(" << sc.x_adv << "," << sc.y_adv << ")"
	           << "  along=" << along << "  lateral=" << lateral
	           << "  expected=" << toStr(sc.expected)
	           << "  actual="   << toStr(sc.actual)
	           << "  [" << verdict << "]" << logs::end;
}

// Genere le SVG d'une scene dans un <g transform> local
std::string svgScene(const Scene& sc, float scale, float cellW, float cellH)
{
	std::ostringstream svg;

	// Cadre de la cellule
	svg << "<rect x='0' y='0' width='" << cellW << "' height='" << cellH
	    << "' fill='#f8f8f8' stroke='#ccc' stroke-width='1'/>\n";

	// Origine locale : on centre le segment dans la cellule
	float margin = 40.0f;
	float minX = std::min({sc.x_robot, sc.x_target, sc.x_adv - ADV_RADIUS}) - ROBOT_RADIUS;
	float maxX = std::max({sc.x_robot, sc.x_target, sc.x_adv + ADV_RADIUS}) + ROBOT_RADIUS;
	float minY = std::min({sc.y_robot, sc.y_target, sc.y_adv - ADV_RADIUS}) - ROBOT_RADIUS;
	float maxY = std::max({sc.y_robot, sc.y_target, sc.y_adv + ADV_RADIUS}) + ROBOT_RADIUS;
	float spanX = (maxX - minX) * scale;
	float spanY = (maxY - minY) * scale;
	float offX = margin + (cellW - 2*margin - spanX) * 0.5f - minX * scale;
	float offY = margin + (cellH - 2*margin - spanY) * 0.5f - minY * scale;

	svg << "<g transform='translate(" << offX << "," << offY << ") scale(" << scale << ")'>\n";

	// Couloir : rectangle oriente le long du segment, largeur = corridor_width
	float dx = sc.x_target - sc.x_robot;
	float dy = sc.y_target - sc.y_robot;
	float len = std::sqrt(dx*dx + dy*dy);
	if (len > 0.001f) {
		float angleDeg = std::atan2(dy, dx) * 180.0f / (float)M_PI;
		float halfW = CORRIDOR_WIDTH * 0.5f;

		svg << "<g transform='translate(" << sc.x_robot << "," << sc.y_robot
		    << ") rotate(" << angleDeg << ")'>\n";
		// Zone APPROACHING (slow)
		svg << "<rect x='" << STOP_DIST << "' y='" << -halfW
		    << "' width='" << (SLOW_DIST - STOP_DIST) << "' height='" << CORRIDOR_WIDTH
		    << "' fill='#FFA500' fill-opacity='0.15' stroke='none'/>\n";
		// Zone BLOCKING (stop)
		svg << "<rect x='0' y='" << -halfW
		    << "' width='" << STOP_DIST << "' height='" << CORRIDOR_WIDTH
		    << "' fill='#DC143C' fill-opacity='0.18' stroke='none'/>\n";
		// Couloir entier (contour seul)
		svg << "<rect x='0' y='" << -halfW
		    << "' width='" << len << "' height='" << CORRIDOR_WIDTH
		    << "' fill='none' stroke='#8888aa' stroke-width='3' stroke-dasharray='20,10'/>\n";
		svg << "</g>\n";
	}

	// Segment robot -> cible
	svg << "<line x1='" << sc.x_robot << "' y1='" << sc.y_robot
	    << "' x2='" << sc.x_target << "' y2='" << sc.y_target
	    << "' stroke='#0066cc' stroke-width='5'/>\n";

	// Projection perpendiculaire adv -> segment (trait gris pointille)
	if (len > 0.001f) {
		float t = ((sc.x_adv - sc.x_robot) * dx + (sc.y_adv - sc.y_robot) * dy) / (len*len);
		float tc = std::max(0.0f, std::min(1.0f, t));
		float px = sc.x_robot + tc * dx;
		float py = sc.y_robot + tc * dy;
		svg << "<line x1='" << sc.x_adv << "' y1='" << sc.y_adv
		    << "' x2='" << px << "' y2='" << py
		    << "' stroke='#888' stroke-width='3' stroke-dasharray='8,6'/>\n";
	}

	// Robot
	svg << "<circle cx='" << sc.x_robot << "' cy='" << sc.y_robot
	    << "' r='" << ROBOT_RADIUS << "' fill='#4A90E2' fill-opacity='0.35' stroke='#0066cc' stroke-width='4'/>\n";
	svg << "<circle cx='" << sc.x_robot << "' cy='" << sc.y_robot
	    << "' r='20' fill='#0066cc'/>\n";

	// Cible
	svg << "<circle cx='" << sc.x_target << "' cy='" << sc.y_target
	    << "' r='30' fill='none' stroke='#0066cc' stroke-width='5'/>\n";
	svg << "<line x1='" << (sc.x_target - 40) << "' y1='" << (sc.y_target - 40)
	    << "' x2='" << (sc.x_target + 40) << "' y2='" << (sc.y_target + 40)
	    << "' stroke='#0066cc' stroke-width='4'/>\n";
	svg << "<line x1='" << (sc.x_target - 40) << "' y1='" << (sc.y_target + 40)
	    << "' x2='" << (sc.x_target + 40) << "' y2='" << (sc.y_target - 40)
	    << "' stroke='#0066cc' stroke-width='4'/>\n";

	// Adversaire
	svg << "<circle cx='" << sc.x_adv << "' cy='" << sc.y_adv
	    << "' r='" << ADV_RADIUS << "' fill='#DC143C' fill-opacity='0.30' stroke='#8B0000' stroke-width='4'/>\n";
	svg << "<circle cx='" << sc.x_adv << "' cy='" << sc.y_adv
	    << "' r='20' fill='#8B0000'/>\n";

	svg << "</g>\n";

	// Labels en pied de cellule (pas scales)
	bool pass = (sc.expected == sc.actual);
	svg << "<text x='" << margin << "' y='22' font-family='monospace' font-size='14' font-weight='bold'>"
	    << sc.name << "</text>\n";
	svg << "<text x='" << margin << "' y='" << (cellH - 26)
	    << "' font-family='monospace' font-size='12'>expected: <tspan fill='"
	    << svgFill(sc.expected) << "' font-weight='bold'>" << toStr(sc.expected)
	    << "</tspan></text>\n";
	svg << "<text x='" << margin << "' y='" << (cellH - 10)
	    << "' font-family='monospace' font-size='12'>actual: <tspan fill='"
	    << svgFill(sc.actual) << "' font-weight='bold'>" << toStr(sc.actual)
	    << "</tspan>  <tspan fill='" << (pass ? "#228B22" : "#DC143C") << "' font-weight='bold'>"
	    << (pass ? "[PASS]" : "[FAIL]") << "</tspan></text>\n";

	return svg.str();
}

void writeSvgFile()
{
	const logs::Logger& log = logs::LoggerFactory::logger("IsOnPathTest");

	const int cols = 2;
	const int rows = (g_scenes.size() + cols - 1) / cols;
	const float cellW = 600.0f;
	const float cellH = 450.0f;
	const float scale = 0.20f;   // 1mm -> 0.20px
	const float totalW = cols * cellW;
	const float totalH = rows * cellH + 60;  // +titre

	std::ostringstream svg;
	svg << "<?xml version='1.0' encoding='UTF-8'?>\n";
	svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << totalW
	    << "' height='" << totalH << "' viewBox='0 0 " << totalW << " " << totalH << "'>\n";

	svg << "<text x='10' y='30' font-family='monospace' font-size='20' font-weight='bold'>"
	    << "IsOnPath tests  (corridor=" << CORRIDOR_WIDTH
	    << "mm  slow=" << SLOW_DIST << "mm  stop=" << STOP_DIST << "mm)</text>\n";
	svg << "<text x='10' y='50' font-family='monospace' font-size='11'>"
	    << "bleu=robot+cible  rouge=adv  zone rouge=BLOCKING  orange=APPROACHING  pointille=couloir</text>\n";

	for (size_t i = 0; i < g_scenes.size(); i++) {
		int r = i / cols;
		int c = i % cols;
		float x = c * cellW;
		float y = 60 + r * cellH;
		svg << "<g transform='translate(" << x << "," << y << ")'>\n";
		svg << svgScene(g_scenes[i], scale, cellW, cellH);
		svg << "</g>\n";
	}

	svg << "</svg>\n";

	std::string path = exeDirectory() + "/isonpath_tests.svg";
	std::ofstream ofs(path);
	if (ofs) {
		ofs << svg.str();
		log.info() << "SVG de validation ecrit : " << path << logs::end;
	} else {
		log.error() << "impossible d'ecrire le SVG : " << path << logs::end;
	}
}

} // namespace

void test::IsOnPathTest::suite()
{
	g_scenes.clear();

	testClear_AdvFarFromPath();
	testClear_AdvBehindRobot();
	testClear_AdvBeyondTarget();

	testApproaching_InCorridorBetweenSlowAndStop();

	testBlocking_InCorridorCloserThanStop();
	testBlocking_AdvOnTarget();

	testClear_ZeroLengthSegment();
	testBoundary_ExactlyAtStopDistance();
	testBoundary_ExactlyAtCorridorEdge();
	testDiagonalPath();

	testDiagonal_NegativeDirection();
	testDiagonal_OddAngle();
	testDiagonal_45_AdvBesideCorridor();
	testDiagonal_45_LateralExactlyHalfW();

	// Log ASCII de toutes les scenes
	logger().info() << "===== IsOnPath scenes =====" << logs::end;
	for (const auto& sc : g_scenes) {
		logScene(sc);
	}

	// SVG a cote du binaire
	writeSvgFile();
}

// ---------- CLEAR : adv hors couloir ou hors segment ----------

void test::IsOnPathTest::testClear_AdvFarFromPath()
{
	PathStatus ps = runScene("Clear_AdvFarFromPath",
			0.0f, 0.0f, 1000.0f, 0.0f, 500.0f, 500.0f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"adv a 500mm lateral (hors couloir 680) => CLEAR");
}

void test::IsOnPathTest::testClear_AdvBehindRobot()
{
	PathStatus ps = runScene("Clear_AdvBehindRobot",
			0.0f, 0.0f, 1000.0f, 0.0f, -200.0f, 0.0f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"adv derriere robot (projection t<0) => CLEAR");
}

void test::IsOnPathTest::testClear_AdvBeyondTarget()
{
	PathStatus ps = runScene("Clear_AdvBeyondTarget",
			0.0f, 0.0f, 1000.0f, 0.0f, 1200.0f, 0.0f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"adv au-dela cible (projection t>1) => CLEAR");
}

// ---------- APPROACHING : adv dans couloir, entre stop et slow ----------

void test::IsOnPathTest::testApproaching_InCorridorBetweenSlowAndStop()
{
	PathStatus ps = runScene("Approaching_InCorridor",
			0.0f, 0.0f, 2000.0f, 0.0f, 550.0f, 100.0f,
			PathStatus::APPROACHING);
	this->assert(ps == PathStatus::APPROACHING,
			"adv along=550mm (entre stop 460 et slow 620), dans couloir => APPROACHING");
}

// ---------- BLOCKING : adv dans couloir, distance <= stop ----------

void test::IsOnPathTest::testBlocking_InCorridorCloserThanStop()
{
	PathStatus ps = runScene("Blocking_InCorridor",
			0.0f, 0.0f, 2000.0f, 0.0f, 300.0f, 50.0f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"adv along=300mm (< stop 460), dans couloir => BLOCKING");
}

void test::IsOnPathTest::testBlocking_AdvOnTarget()
{
	PathStatus ps = runScene("Blocking_AdvOnTarget",
			0.0f, 0.0f, 400.0f, 0.0f, 400.0f, 0.0f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"adv pile sur cible (segment 400mm, stop 460) => BLOCKING");
}

// ---------- Cas particuliers ----------

void test::IsOnPathTest::testClear_ZeroLengthSegment()
{
	PathStatus ps1 = runScene("Clear_ZeroSegment_AdvNear",
			500.0f, 500.0f, 500.0f, 500.0f, 500.0f, 600.0f,
			PathStatus::CLEAR);
	this->assert(ps1 == PathStatus::CLEAR,
			"segment nul (rotation pure) + adv proche => CLEAR");

	PathStatus ps2 = runScene("Clear_ZeroSegment_AdvOnRobot",
			500.0f, 500.0f, 500.0f, 500.0f, 500.0f, 500.0f,
			PathStatus::CLEAR);
	this->assert(ps2 == PathStatus::CLEAR,
			"segment nul + adv pile sur robot => CLEAR");
}

void test::IsOnPathTest::testBoundary_ExactlyAtStopDistance()
{
	PathStatus ps = runScene("Boundary_AtStopDistance",
			0.0f, 0.0f, 1000.0f, 0.0f, 460.0f, 0.0f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"adv along = stop pile (460mm) => BLOCKING (inclusif)");
}

void test::IsOnPathTest::testBoundary_ExactlyAtCorridorEdge()
{
	PathStatus ps = runScene("Boundary_AtCorridorEdge",
			0.0f, 0.0f, 1000.0f, 0.0f, 300.0f, 340.0f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"adv lateral = corridor/2 pile (340mm) => CLEAR (hors couloir)");
}

void test::IsOnPathTest::testDiagonalPath()
{
	PathStatus ps = runScene("Diagonal_AdvOnSegment",
			0.0f, 0.0f, 1000.0f, 1000.0f, 200.0f, 200.0f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"segment diagonal, adv pile sur segment a 283mm => BLOCKING");
}

// ---------- Diagonales supplementaires (validation geometrie) ----------

void test::IsOnPathTest::testDiagonal_NegativeDirection()
{
	// Segment oriente sud-ouest : robot (1000,1000) -> cible (0,0)
	// Adv (800, 800) : pile sur segment, t = 0.2, along = 0.2 * sqrt(2)*1000 = 283 mm
	// along < stop (460) => BLOCKING
	// Verifie que dx, dy negatifs sont bien traites.
	PathStatus ps = runScene("Diagonal_NegativeDirection",
			1000.0f, 1000.0f, 0.0f, 0.0f, 800.0f, 800.0f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"segment diagonal sud-ouest, adv pile a 283mm => BLOCKING");
}

void test::IsOnPathTest::testDiagonal_OddAngle()
{
	// Segment d'angle arbitraire (~26.6 deg) : robot (0,0) -> cible (1000, 500).
	// longueur = sqrt(1250000) ~ 1118.03 mm.
	// Adv pile a along = 300 mm : t = 300 / 1118 ~ 0.2683
	// projection = (268.33, 134.16), adv = projection (lateral = 0)
	// along < stop (460) => BLOCKING
	PathStatus ps = runScene("Diagonal_OddAngle",
			0.0f, 0.0f, 1000.0f, 500.0f, 268.33f, 134.16f,
			PathStatus::BLOCKING);
	this->assert(ps == PathStatus::BLOCKING,
			"segment angle arbitraire, adv pile sur segment => BLOCKING");
}

void test::IsOnPathTest::testDiagonal_45_AdvBesideCorridor()
{
	// Segment 45 deg : robot (0,0) -> cible (1000, 1000).
	// Adv (0, 500) : projection = (250, 250), lateral = sqrt(250^2 + 250^2) = 353.55 mm
	// lateral > corridor/2 (340) => hors couloir => CLEAR
	PathStatus ps = runScene("Diagonal_45_AdvBesideCorridor",
			0.0f, 0.0f, 1000.0f, 1000.0f, 0.0f, 500.0f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"diagonale 45deg, adv lateral 353mm (> corridor/2=340) => CLEAR");
}

void test::IsOnPathTest::testDiagonal_45_LateralExactlyHalfW()
{
	// Segment 45 deg : robot (0,0) -> cible (1000, 1000).
	// Vecteur perpendiculaire unitaire : (-1/sqrt(2), 1/sqrt(2)) ~ (-0.7071, 0.7071)
	// Projection (200, 200) + half_w * perp = (200 - 240.42, 200 + 240.42) = (-40.42, 440.42)
	// lateral = 340 pile => CLEAR (limite exclusive)
	PathStatus ps = runScene("Diagonal_45_LateralExactlyHalfW",
			0.0f, 0.0f, 1000.0f, 1000.0f, -40.42f, 440.42f,
			PathStatus::CLEAR);
	this->assert(ps == PathStatus::CLEAR,
			"diagonale 45deg, adv lateral = corridor/2 pile (340mm) => CLEAR (exclusif)");
}
