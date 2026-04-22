#ifndef TEST_STRATEGYJSONPARSER_TEST_HPP
#define TEST_STRATEGYJSONPARSER_TEST_HPP

#include "../suite/UnitTest.hpp"

namespace test {

class StrategyJsonParserTest : public UnitTest {
public:
    StrategyJsonParserTest() : UnitTest("StrategyJsonParserTest") {}
    virtual ~StrategyJsonParserTest() {}

    virtual void suite();

    void testLoadBasicFields();
    void testPrioritySortDesc();
    void testStableSortWhenEqualPriority();
    void testMissingPriorityDefaultsToZero();
    void testFlagsFieldsParsed();
    void testLoadBadFileReturnsFalse();
};

}

#endif
