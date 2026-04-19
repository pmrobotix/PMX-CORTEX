/*!
 * \file
 * \brief Test unitaire du predicat ObstacleZone::isOnPath.
 *
 * Teste la classification d'un adversaire (CLEAR/APPROACHING/BLOCKING)
 * par rapport au segment [robot -> cible]. Logique pure, aucun driver requis.
 *
 * Contexte TDD : ces tests sont ecrits AVANT l'implementation du predicat.
 * En l'etat (stub retournant CLEAR), seuls les tests attendant CLEAR passent.
 * Les tests attendant APPROACHING/BLOCKING echouent jusqu'a T3.
 */

#ifndef TEST_ISONPATHTEST_HPP
#define TEST_ISONPATHTEST_HPP

#include "../suite/UnitTest.hpp"
#include "log/LoggerFactory.hpp"

namespace test {

class IsOnPathTest : public UnitTest
{
private:

	static inline const logs::Logger & logger()
	{
		static const logs::Logger & instance = logs::LoggerFactory::logger("IsOnPathTest");
		return instance;
	}

public:

	IsOnPathTest() : UnitTest("IsOnPathTest") {}
	virtual ~IsOnPathTest() {}

	virtual void suite();

	// CLEAR : adv hors couloir ou hors segment
	void testClear_AdvFarFromPath();
	void testClear_AdvBehindRobot();
	void testClear_AdvBeyondTarget();

	// APPROACHING : adv dans couloir, entre stop et slow
	void testApproaching_InCorridorBetweenSlowAndStop();

	// BLOCKING : adv dans couloir, distance <= stop
	void testBlocking_InCorridorCloserThanStop();
	void testBlocking_AdvOnTarget();

	// Cas particuliers
	void testClear_ZeroLengthSegment();
	void testBoundary_ExactlyAtStopDistance();
	void testBoundary_ExactlyAtCorridorEdge();
	void testDiagonalPath();

	// Diagonales supplementaires (validation geometrie)
	void testDiagonal_NegativeDirection();
	void testDiagonal_OddAngle();
	void testDiagonal_45_AdvBesideCorridor();
	void testDiagonal_45_LateralExactlyHalfW();
};

}

#endif
