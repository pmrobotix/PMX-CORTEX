#ifndef TEST_FLAGMANAGER_TEST_HPP
#define TEST_FLAGMANAGER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class FlagManagerTest : public UnitTest {
public:
    FlagManagerTest() : UnitTest("FlagManagerTest") {}
    virtual ~FlagManagerTest() {}

    virtual void suite();

    void testSetAndHas();
    void testClear();
    void testClearAll();
    void testEmptyFlagNameIgnored();
    void testSetIdempotent();
    void testClearMissingFlag();
};

}

#endif
