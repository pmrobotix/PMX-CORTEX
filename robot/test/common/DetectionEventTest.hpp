/*!
 * \file
 * \brief Test unitaire de la structure DetectionEvent.
 *
 * Teste la construction, les helpers (isBlocking, isSlowDown, hasPosition)
 * et le clear. Logique pure, aucune dependance.
 */

#ifndef TEST_DETECTIONEVENTTEST_HPP
#define TEST_DETECTIONEVENTTEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

class DetectionEventTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("DetectionEventTest");
		return instance;
	}

public:

	DetectionEventTest() : UnitTest("DetectionEventTest") {}
	virtual ~DetectionEventTest() {}

	virtual void suite();

	void testDefaultConstruction();
	void testIsBlocking();
	void testIsSlowDown();
	void testHasPosition();
	void testClear();
};

}

#endif
