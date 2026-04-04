/*!
 * \file
 * \brief Tests unitaires de RetryPolicy.
 */

#include "RetryPolicyTest.hpp"
#include "../../src/common/navigator/RetryPolicy.hpp"

void test::RetryPolicyTest::suite()
{
    testNoRetry();
    testStandard();
    testAggressive();
    testPatient();
    testCustom();
}

void test::RetryPolicyTest::testNoRetry()
{
    RetryPolicy p = RetryPolicy::noRetry();
    this->assert(p.waitTempoUs == 0, "noRetry waitTempoUs == 0");
    this->assert(p.maxObstacleRetries == 1, "noRetry maxObstacleRetries == 1");
    this->assert(p.maxCollisionRetries == 1, "noRetry maxCollisionRetries == 1");
    this->assert(p.reculObstacleMm == 0, "noRetry reculObstacleMm == 0");
    this->assert(p.reculCollisionMm == 0, "noRetry reculCollisionMm == 0");
    this->assert(p.rotateIgnoringOpponent == true, "noRetry rotateIgnoringOpponent == true");
    this->assert(p.ignoreCollision == false, "noRetry ignoreCollision == false");
}

void test::RetryPolicyTest::testStandard()
{
    RetryPolicy p = RetryPolicy::standard();
    this->assert(p.waitTempoUs == 2000000, "standard waitTempoUs == 2000000");
    this->assert(p.maxObstacleRetries == 2, "standard maxObstacleRetries == 2");
    this->assert(p.maxCollisionRetries == 2, "standard maxCollisionRetries == 2");
    this->assert(p.reculObstacleMm == 0, "standard reculObstacleMm == 0");
    this->assert(p.reculCollisionMm == 0, "standard reculCollisionMm == 0");
}

void test::RetryPolicyTest::testAggressive()
{
    RetryPolicy p = RetryPolicy::aggressive();
    this->assert(p.waitTempoUs == 2000000, "aggressive waitTempoUs == 2000000");
    this->assert(p.maxObstacleRetries == 5, "aggressive maxObstacleRetries == 5");
    this->assert(p.maxCollisionRetries == 5, "aggressive maxCollisionRetries == 5");
    this->assert(p.reculObstacleMm == 50, "aggressive reculObstacleMm == 50");
    this->assert(p.reculCollisionMm == 20, "aggressive reculCollisionMm == 20");
}

void test::RetryPolicyTest::testPatient()
{
    RetryPolicy p = RetryPolicy::patient();
    this->assert(p.maxObstacleRetries == 20, "patient maxObstacleRetries == 20");
    this->assert(p.maxCollisionRetries == 10, "patient maxCollisionRetries == 10");
    this->assert(p.reculObstacleMm == 0, "patient reculObstacleMm == 0");
}

void test::RetryPolicyTest::testCustom()
{
    RetryPolicy p = RetryPolicy::standard();
    p.maxObstacleRetries = 10;
    p.reculObstacleMm = 100;

    // Le custom ne doit pas affecter les presets
    RetryPolicy s = RetryPolicy::standard();
    this->assert(s.maxObstacleRetries == 2, "custom ne modifie pas standard maxObstacleRetries");
    this->assert(s.reculObstacleMm == 0, "custom ne modifie pas standard reculObstacleMm");

    // Le custom doit avoir ses valeurs
    this->assert(p.maxObstacleRetries == 10, "custom maxObstacleRetries == 10");
    this->assert(p.reculObstacleMm == 100, "custom reculObstacleMm == 100");
    // Les autres champs restent standard
    this->assert(p.maxCollisionRetries == 2, "custom garde maxCollisionRetries standard");
}
