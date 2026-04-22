#include "FlagManagerTest.hpp"
#include "../../src/common/ia/FlagManager.hpp"

void test::FlagManagerTest::suite()
{
    testSetAndHas();
    testClear();
    testClearAll();
    testEmptyFlagNameIgnored();
    testSetIdempotent();
    testClearMissingFlag();
}

void test::FlagManagerTest::testSetAndHas()
{
    FlagManager f;
    this->assert(!f.has("x"), "has returns false before set");
    f.set("x");
    this->assert(f.has("x"), "has returns true after set");
    this->assert(!f.has("y"), "unrelated flag stays false");
}

void test::FlagManagerTest::testClear()
{
    FlagManager f;
    f.set("a");
    f.set("b");
    f.clear("a");
    this->assert(!f.has("a"), "a cleared");
    this->assert(f.has("b"), "b still set");
    this->assert(f.size() == 1, "size == 1 after clearing a");
}

void test::FlagManagerTest::testClearAll()
{
    FlagManager f;
    f.set("a");
    f.set("b");
    f.set("c");
    this->assert(f.size() == 3, "size == 3 before clearAll");
    f.clearAll();
    this->assert(f.size() == 0, "size == 0 after clearAll");
    this->assert(!f.has("a") && !f.has("b") && !f.has("c"), "all flags gone after clearAll");
}

void test::FlagManagerTest::testEmptyFlagNameIgnored()
{
    FlagManager f;
    f.set("");
    this->assert(f.size() == 0, "set(\"\") ignore");
    this->assert(!f.has(""), "has(\"\") false");
    f.set("real");
    f.clear("");
    this->assert(f.has("real"), "clear(\"\") ne touche pas aux autres");
}

void test::FlagManagerTest::testSetIdempotent()
{
    FlagManager f;
    f.set("x");
    f.set("x");
    f.set("x");
    this->assert(f.size() == 1, "set multiple fois ne duplique pas");
    this->assert(f.has("x"), "x toujours present");
}

void test::FlagManagerTest::testClearMissingFlag()
{
    FlagManager f;
    f.set("a");
    f.clear("absent");
    this->assert(f.has("a"), "clear d'un flag absent ne casse rien");
    this->assert(f.size() == 1, "size reste 1");
}
