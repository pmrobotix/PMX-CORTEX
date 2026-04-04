/*!
 * \file
 * \brief Definition de la classe RetryPolicyTest.
 */

#ifndef TEST_RETRYPOLICY_TEST_HPP
#define TEST_RETRYPOLICY_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class RetryPolicyTest : public UnitTest {
public:
    RetryPolicyTest() : UnitTest("RetryPolicyTest") {}
    virtual ~RetryPolicyTest() {}

    virtual void suite();

    void testNoRetry();
    void testStandard();
    void testAggressive();
    void testPatient();
    void testCustom();
};

}

#endif
