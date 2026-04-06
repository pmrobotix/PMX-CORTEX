/*!
 * \file
 * \brief Test unitaire de la classe TableGeometry.
 *
 * Teste les filtres d'appartenance au terrain : isPointInsideTable()
 * et isSensorReadingInsideTable(). Logique pure, aucun driver requis.
 */

#ifndef TEST_TABLEGEOMETRYTEST_HPP
#define TEST_TABLEGEOMETRYTEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

/*!
 * \brief Test unitaire de la geometrie de table.
 */
class TableGeometryTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("TableGeometryTest");
		return instance;
	}

public:

	TableGeometryTest() : UnitTest("TableGeometryTest") {}
	virtual ~TableGeometryTest() {}

	virtual void suite();

	// isPointInsideTable
	void testPointInside();
	void testPointOutsideLeft();
	void testPointOutsideRight();
	void testPointOutsideTop();
	void testPointOutsideBottom();
	void testPointOnMargin();
	void testPointAtCenter();
	void testPointNegative();

	// isSensorReadingInsideTable
	void testSensorFrontInside();
	void testSensorFrontOutside();
	void testSensorBehindInside();

	// Accesseurs
	void testAccessors();
};

}

#endif
