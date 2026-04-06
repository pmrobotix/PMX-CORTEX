/*!
 * \file
 * \brief Implementation du test unitaire TableGeometry.
 *
 * Table 3000x2000 avec marge 90mm (Coupe de France).
 * Points valides : x dans ]90, 2910[, y dans ]90, 1910[.
 *
 * Utilise un stub local de ARobotPositionShared pour eviter
 * la dependance au driver (singleton). Ainsi ce test reste dans common.
 */

#include "TableGeometryTest.hpp"

#include "../../src/common/geometry/TableGeometry.hpp"
#include "../../src/common/interface/ARobotPositionShared.hpp"

// Stub minimal de ARobotPositionShared (pas de mutex, pas de thread)
class StubRobotPosition : public ARobotPositionShared {
	ROBOTPOSITION pos_;
public:
	StubRobotPosition() : pos_{0, 0, 0, 0, 0, 0} {}
	ROBOTPOSITION getRobotPosition(int) override { return pos_; }
	void setRobotPosition(ROBOTPOSITION p) override { pos_ = p; }
};

static StubRobotPosition stubPos_;

void test::TableGeometryTest::suite()
{
	testPointInside();
	testPointOutsideLeft();
	testPointOutsideRight();
	testPointOutsideTop();
	testPointOutsideBottom();
	testPointOnMargin();
	testPointAtCenter();
	testPointNegative();

	testSensorFrontInside();
	testSensorFrontOutside();
	testSensorBehindInside();

	testAccessors();
}

// ---- isPointInsideTable ----

void test::TableGeometryTest::testPointInside()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(tg.isPointInsideTable(500, 500), "point (500,500) inside table");
	this->assert(tg.isPointInsideTable(1500, 1000), "point (1500,1000) centre inside table");
}

void test::TableGeometryTest::testPointOutsideLeft()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(!tg.isPointInsideTable(50, 1000), "point (50,1000) outside left");
}

void test::TableGeometryTest::testPointOutsideRight()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(!tg.isPointInsideTable(2950, 1000), "point (2950,1000) outside right");
}

void test::TableGeometryTest::testPointOutsideTop()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(!tg.isPointInsideTable(1500, 1950), "point (1500,1950) outside top");
}

void test::TableGeometryTest::testPointOutsideBottom()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(!tg.isPointInsideTable(1500, 50), "point (1500,50) outside bottom");
}

void test::TableGeometryTest::testPointOnMargin()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	// Exactement sur la marge : x==90 => !(x > 90) => false
	this->assert(!tg.isPointInsideTable(90, 1000), "point (90,1000) sur marge => outside");
	this->assert(!tg.isPointInsideTable(2910, 1000), "point (2910,1000) sur marge => outside");
	this->assert(!tg.isPointInsideTable(1500, 90), "point (1500,90) sur marge => outside");
	this->assert(!tg.isPointInsideTable(1500, 1910), "point (1500,1910) sur marge => outside");

	// Juste apres la marge : x==91 => inside
	this->assert(tg.isPointInsideTable(91, 91), "point (91,91) juste apres marge => inside");
	this->assert(tg.isPointInsideTable(2909, 1909), "point (2909,1909) juste avant marge => inside");
}

void test::TableGeometryTest::testPointAtCenter()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(tg.isPointInsideTable(1500, 1000), "centre table (1500,1000) inside");
}

void test::TableGeometryTest::testPointNegative()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(!tg.isPointInsideTable(-100, 500), "point negatif x => outside");
	this->assert(!tg.isPointInsideTable(500, -100), "point negatif y => outside");
}

// ---- isSensorReadingInsideTable ----

void test::TableGeometryTest::testSensorFrontInside()
{
	// Robot au centre, angle 0 (face vers x+)
	ROBOTPOSITION rp = {1500.0f, 1000.0f, 0.0f, 0, 0, 0};
	stubPos_.setRobotPosition(rp);
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	// Capteur centre, distance 500mm devant => (2000, 1000) : inside
	this->assert(tg.isSensorReadingInsideTable(500, 0),
			"capteur centre 500mm devant depuis centre => inside");
}

void test::TableGeometryTest::testSensorFrontOutside()
{
	// Robot pres du bord droit, angle 0
	ROBOTPOSITION rp = {2800.0f, 1000.0f, 0.0f, 0, 0, 0};
	stubPos_.setRobotPosition(rp);
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	// Capteur centre, distance 500mm devant => (3300, 1000) : outside
	this->assert(!tg.isSensorReadingInsideTable(500, 0),
			"capteur 500mm devant depuis bord droit => outside");
}

void test::TableGeometryTest::testSensorBehindInside()
{
	// Robot au centre, angle 0
	ROBOTPOSITION rp = {1500.0f, 1000.0f, 0.0f, 0, 0, 0};
	stubPos_.setRobotPosition(rp);
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	// Capteur centre, distance negative = derriere => (1000, 1000) : inside
	this->assert(tg.isSensorReadingInsideTable(-500, 0),
			"capteur centre -500mm (arriere) depuis centre => inside");
}

// ---- Accesseurs ----

void test::TableGeometryTest::testAccessors()
{
	TableGeometry tg(3000, 2000, 90, &stubPos_);

	this->assert(tg.widthMm() == 3000, "widthMm() == 3000");
	this->assert(tg.heightMm() == 2000, "heightMm() == 2000");
	this->assert(tg.marginMm() == 90, "marginMm() == 90");
}
