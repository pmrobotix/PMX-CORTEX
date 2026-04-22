#include "StrategyJsonParserTest.hpp"
#include "../../src/common/ia/StrategyJsonParser.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

// Helper : ecrit un JSON temporaire, renvoie le chemin. Le fichier est
// supprime par le destructeur.
class TempJson {
public:
    TempJson(const std::string& content)
    {
        path_ = std::string("/tmp/strategyJsonParserTest_") + std::to_string(::getpid())
                + "_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".json";
        std::ofstream f(path_);
        f << content;
    }
    ~TempJson() { std::remove(path_.c_str()); }

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

} // namespace

void test::StrategyJsonParserTest::suite()
{
    testLoadBasicFields();
    testPrioritySortDesc();
    testStableSortWhenEqualPriority();
    testMissingPriorityDefaultsToZero();
    testFlagsFieldsParsed();
    testLoadBadFileReturnsFalse();
}

void test::StrategyJsonParserTest::testLoadBasicFields()
{
    TempJson tmp(R"([
        { "id": 42, "desc": "hello", "priority": 7.5, "points": 20,
          "estimatedDurationSec": 3.5, "tasks": [] }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v.size() == 1, "1 instruction chargee");
    const auto& i = v[0];
    this->assert(i.id == 42, "id==42");
    this->assert(i.desc == "hello", "desc==hello");
    this->assert(i.priority == 7.5f, "priority==7.5");
    this->assert(i.points.has_value() && *i.points == 20, "points==20");
    this->assert(i.estimatedDurationSec.has_value() && *i.estimatedDurationSec == 3.5f,
                 "EDSec==3.5");
}

void test::StrategyJsonParserTest::testPrioritySortDesc()
{
    TempJson tmp(R"([
        { "id": 1, "priority": 5,   "tasks": [] },
        { "id": 2, "priority": 100, "tasks": [] },
        { "id": 3, "priority": 50,  "tasks": [] }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v.size() == 3, "3 instructions");
    this->assert(v[0].id == 2 && v[0].priority == 100.0f, "ordre[0]=id2 priority=100");
    this->assert(v[1].id == 3 && v[1].priority == 50.0f,  "ordre[1]=id3 priority=50");
    this->assert(v[2].id == 1 && v[2].priority == 5.0f,   "ordre[2]=id1 priority=5");
}

void test::StrategyJsonParserTest::testStableSortWhenEqualPriority()
{
    TempJson tmp(R"([
        { "id": 1, "priority": 10, "tasks": [] },
        { "id": 2, "priority": 10, "tasks": [] },
        { "id": 3, "priority": 10, "tasks": [] }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v[0].id == 1 && v[1].id == 2 && v[2].id == 3,
                 "ordre JSON conserve a priorite egale (stable_sort)");
}

void test::StrategyJsonParserTest::testMissingPriorityDefaultsToZero()
{
    TempJson tmp(R"([
        { "id": 1, "tasks": [] },
        { "id": 2, "priority": -1, "tasks": [] },
        { "id": 3, "priority": 1,  "tasks": [] }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    // ordre attendu : id=3 (p=1), id=1 (p=0), id=2 (p=-1)
    this->assert(v[0].id == 3 && v[0].priority == 1.0f,  "top = id3 priority 1");
    this->assert(v[1].id == 1 && v[1].priority == 0.0f,  "mid = id1 priority 0 (defaut)");
    this->assert(v[2].id == 2 && v[2].priority == -1.0f, "bot = id2 priority -1");
}

void test::StrategyJsonParserTest::testFlagsFieldsParsed()
{
    TempJson tmp(R"([
        { "id": 1, "needed_flag": "go", "action_flag": "done",
          "clear_flags": ["a", "b"], "tasks": [] }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    const auto& i = v[0];
    this->assert(i.needed_flag.has_value() && *i.needed_flag == "go", "needed_flag parse");
    this->assert(i.action_flag.has_value() && *i.action_flag == "done", "action_flag parse");
    this->assert(i.clear_flags.size() == 2 && i.clear_flags[0] == "a" && i.clear_flags[1] == "b",
                 "clear_flags parse");
}

void test::StrategyJsonParserTest::testLoadBadFileReturnsFalse()
{
    std::vector<StrategyInstruction> v;
    this->assert(!parseStrategyFromFile("/tmp/this_file_absolutely_does_not_exist_12345.json", v),
                 "parseStrategyFromFile retourne false sur fichier absent");
    this->assert(v.empty(), "vecteur vide apres echec");
}
