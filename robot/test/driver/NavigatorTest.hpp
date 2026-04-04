/*!
 * \file
 * \brief Definition de la classe NavigatorTest.
 */

#ifndef TEST_NAVIGATOR_TEST_HPP
#define TEST_NAVIGATOR_TEST_HPP

#ifdef SIMU

#include "../suite/UnitTest.hpp"

namespace test {

/*!
 * \brief Tests unitaires de Navigator (simulation uniquement).
 *
 * Teste les mouvements simples, rotations, waypoints et PathMode.
 * Necessite le driver simu pour instancier Robot + Asserv.
 */
class NavigatorTest : public UnitTest {
public:
    NavigatorTest() : UnitTest("NavigatorTest") {}
    virtual ~NavigatorTest() {}

    virtual void suite();

    void testConstructor();
    void testConstructorWithoutIAbyPath();
    void testManualPathEmpty();
    void testRetryPolicyDefault();
    void testPathModeEnum();

private:
    void initRobot();
};

}

#endif // SIMU
#endif
