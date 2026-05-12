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
    testNeededAdvOutOfZoneParsed();
    testTaskFieldsParsed();
    testInstructionTimeGatesParsed();
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

void test::StrategyJsonParserTest::testNeededAdvOutOfZoneParsed()
{
    // i1 sans champ -> nullopt. i2 avec rect valide -> set. i3 avec min>max ->
    // ignore (nullopt), parser warn mais ne fail pas le chargement.
    TempJson tmp(R"([
        { "id": 1, "tasks": [] },
        { "id": 2, "tasks": [],
          "needed_adv_out_of_zone": {"x_min":1700,"y_min":700,"x_max":2000,"y_max":1100} },
        { "id": 3, "tasks": [],
          "needed_adv_out_of_zone": {"x_min":2000,"y_min":700,"x_max":1700,"y_max":1100} }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v.size() == 3, "3 instructions chargees");
    // Note : trie par priorite stable, toutes a 0 -> ordre JSON preserve
    const auto* i1 = &v[0]; const auto* i2 = &v[1]; const auto* i3 = &v[2];
    this->assert(i1->id == 1 && !i1->needed_adv_out_of_zone.has_value(),
                 "i1 sans champ -> nullopt");
    this->assert(i2->id == 2 && i2->needed_adv_out_of_zone.has_value(),
                 "i2 avec champ -> set");
    if (i2->needed_adv_out_of_zone) {
        const auto& r = *i2->needed_adv_out_of_zone;
        this->assert(r.x_min == 1700.0f && r.x_max == 2000.0f
                  && r.y_min == 700.0f  && r.y_max == 1100.0f,
                     "i2 rect parse: x_min=1700 x_max=2000 y_min=700 y_max=1100");
    }
    this->assert(i3->id == 3 && !i3->needed_adv_out_of_zone.has_value(),
                 "i3 rect min>max -> ignore (nullopt)");
}

void test::StrategyJsonParserTest::testTaskFieldsParsed()
{
    // Une instruction unique avec une task par categorie (type x subtype) qui
    // exerce chaque champ JSON lu par le parser. Verifie type/subtype/desc/
    // needed_flag/timeout/chain et les champs specifiques (position_x/y, dist,
    // angle_deg, final_angle_deg, rotate_rel_deg, face_x/y, forward,
    // turn_right, action_id, item_id, speed_percent, duration_ms,
    // until_match_sec, waypoints).
    TempJson tmp(R"([
        { "id": 1, "tasks": [
            { "type": "MOVEMENT", "subtype": "LINE", "desc": "recul",
              "needed_flag": "ok", "timeout": 1234, "chain": true, "dist": 150 },
            { "type": "MOVEMENT", "subtype": "GO_TO",
              "position_x": 1500, "position_y": 1000 },
            { "type": "MOVEMENT", "subtype": "ROTATE_DEG", "angle_deg": 45 },
            { "type": "MOVEMENT", "subtype": "GO_TO_AND_FACE_TO",
              "position_x": 800, "position_y": 600,
              "face_x": 0, "face_y": 600 },
            { "type": "MOVEMENT", "subtype": "GO_TO_AND_ROTATE_REL_DEG",
              "position_x": 100, "position_y": 200,
              "rotate_rel_deg": -90, "final_angle_deg": 180 },
            { "type": "MOVEMENT", "subtype": "ORBITAL_TURN_DEG",
              "angle_deg": 60, "forward": false, "turn_right": true },
            { "type": "MOVEMENT", "subtype": "MANUAL_PATH",
              "waypoints": [[100,200],[300,400],[500,600]] },
            { "type": "MANIPULATION", "action_id": "push_elements_P1_B",
              "timeout": 5000 },
            { "type": "ELEMENT", "subtype": "DELETE_ZONE", "item_id": "caisse_1" },
            { "type": "SPEED", "subtype": "SET_SPEED", "speed_percent": 50 },
            { "type": "WAIT", "duration_ms": 500 },
            { "type": "WAIT", "until_match_sec": 75.5 }
        ]}
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v.size() == 1, "1 instruction chargee");
    const auto& tasks = v[0].tasks;
    this->assert(tasks.size() == 12, "12 tasks chargees");

    // t0 : LINE + tous champs communs (desc/needed_flag/timeout/chain) + dist
    const auto& t0 = tasks[0];
    this->assert(t0.type == "MOVEMENT" && t0.subtype == "LINE", "t0 type/subtype");
    this->assert(t0.desc.has_value() && *t0.desc == "recul", "t0 desc==recul");
    this->assert(t0.needed_flag.has_value() && *t0.needed_flag == "ok", "t0 needed_flag");
    this->assert(t0.timeout_ms == 1234, "t0 timeout==1234");
    this->assert(t0.chain == true, "t0 chain==true");
    this->assert(t0.dist.has_value() && *t0.dist == 150.0f, "t0 dist==150");

    // t1 : GO_TO + position_x/y
    const auto& t1 = tasks[1];
    this->assert(t1.subtype == "GO_TO", "t1 subtype==GO_TO");
    this->assert(t1.position_x.has_value() && *t1.position_x == 1500.0f, "t1 position_x==1500");
    this->assert(t1.position_y.has_value() && *t1.position_y == 1000.0f, "t1 position_y==1000");

    // t2 : ROTATE_DEG + angle_deg
    const auto& t2 = tasks[2];
    this->assert(t2.subtype == "ROTATE_DEG" && t2.angle_deg.has_value() && *t2.angle_deg == 45.0f,
                 "t2 angle_deg==45");

    // t3 : GO_TO_AND_FACE_TO + face_x/y
    const auto& t3 = tasks[3];
    this->assert(t3.face_x.has_value() && *t3.face_x == 0.0f, "t3 face_x==0");
    this->assert(t3.face_y.has_value() && *t3.face_y == 600.0f, "t3 face_y==600");

    // t4 : GO_TO_AND_ROTATE_REL_DEG + rotate_rel_deg + final_angle_deg
    const auto& t4 = tasks[4];
    this->assert(t4.rotate_rel_deg.has_value() && *t4.rotate_rel_deg == -90.0f,
                 "t4 rotate_rel_deg==-90");
    this->assert(t4.final_angle_deg.has_value() && *t4.final_angle_deg == 180.0f,
                 "t4 final_angle_deg==180");

    // t5 : ORBITAL_TURN_DEG + forward + turn_right
    const auto& t5 = tasks[5];
    this->assert(t5.forward.has_value() && *t5.forward == false, "t5 forward==false");
    this->assert(t5.turn_right.has_value() && *t5.turn_right == true, "t5 turn_right==true");

    // t6 : MANUAL_PATH + waypoints
    const auto& t6 = tasks[6];
    this->assert(t6.waypoints.size() == 3, "t6 waypoints.size==3");
    if (t6.waypoints.size() == 3) {
        this->assert(t6.waypoints[0][0] == 100.0f && t6.waypoints[0][1] == 200.0f, "t6 wp[0]");
        this->assert(t6.waypoints[1][0] == 300.0f && t6.waypoints[1][1] == 400.0f, "t6 wp[1]");
        this->assert(t6.waypoints[2][0] == 500.0f && t6.waypoints[2][1] == 600.0f, "t6 wp[2]");
    }

    // t7 : MANIPULATION + action_id (+ verif timeout sans champs commun *desc/needed_flag*)
    const auto& t7 = tasks[7];
    this->assert(t7.type == "MANIPULATION", "t7 type==MANIPULATION");
    this->assert(t7.action_id.has_value() && *t7.action_id == "push_elements_P1_B",
                 "t7 action_id");
    this->assert(t7.timeout_ms == 5000, "t7 timeout==5000");
    this->assert(t7.chain == false, "t7 chain default==false");

    // t8 : ELEMENT/DELETE_ZONE + item_id
    const auto& t8 = tasks[8];
    this->assert(t8.type == "ELEMENT" && t8.subtype == "DELETE_ZONE", "t8 type/subtype");
    this->assert(t8.item_id.has_value() && *t8.item_id == "caisse_1", "t8 item_id");

    // t9 : SPEED + speed_percent
    const auto& t9 = tasks[9];
    this->assert(t9.type == "SPEED" && t9.subtype == "SET_SPEED", "t9 type/subtype");
    this->assert(t9.speed_percent.has_value() && *t9.speed_percent == 50, "t9 speed_percent==50");

    // t10 : WAIT + duration_ms
    const auto& t10 = tasks[10];
    this->assert(t10.type == "WAIT", "t10 type==WAIT");
    this->assert(t10.duration_ms.has_value() && *t10.duration_ms == 500, "t10 duration_ms==500");

    // t11 : WAIT + until_match_sec
    const auto& t11 = tasks[11];
    this->assert(t11.until_match_sec.has_value() && *t11.until_match_sec == 75.5f,
                 "t11 until_match_sec==75.5");

    // Verifie aussi les defauts (timeout=-1, chain=false) sur une task minimale
    this->assert(t1.timeout_ms == -1, "t1 timeout default==-1");
    this->assert(t1.chain == false, "t1 chain default==false");
}

void test::StrategyJsonParserTest::testInstructionTimeGatesParsed()
{
    // i1 : sans gates -> nullopt. i2 : avec min+max. i3 : min seul.
    TempJson tmp(R"([
        { "id": 1, "tasks": [] },
        { "id": 2, "tasks": [], "min_match_sec": 10.5, "max_match_sec": 80.0 },
        { "id": 3, "tasks": [], "min_match_sec": 5.0 }
    ])");
    std::vector<StrategyInstruction> v;
    this->assert(parseStrategyFromFile(tmp.path(), v), "parseStrategyFromFile OK");
    this->assert(v.size() == 3, "3 instructions");
    // tri stable, toutes a priorite 0 -> ordre JSON
    const auto& i1 = v[0]; const auto& i2 = v[1]; const auto& i3 = v[2];
    this->assert(!i1.min_match_sec.has_value() && !i1.max_match_sec.has_value(),
                 "i1 sans gates -> nullopt");
    this->assert(i2.min_match_sec.has_value() && *i2.min_match_sec == 10.5f,
                 "i2 min_match_sec==10.5");
    this->assert(i2.max_match_sec.has_value() && *i2.max_match_sec == 80.0f,
                 "i2 max_match_sec==80.0");
    this->assert(i3.min_match_sec.has_value() && *i3.min_match_sec == 5.0f,
                 "i3 min_match_sec==5.0");
    this->assert(!i3.max_match_sec.has_value(),
                 "i3 max_match_sec absent -> nullopt");
}
