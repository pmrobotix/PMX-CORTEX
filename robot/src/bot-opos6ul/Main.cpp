
// TODO: Re-enable test includes once tests are migrated to PMX-CORTEX
#include "O_ActionManagerTimerTest.hpp"
// #include "O_Asserv_CalageTest.hpp"
// #include "O_Asserv_SquareTest.hpp"
// #include "O_AsservEsialTest.hpp"
// #include "O_AsservLineRotateTest.hpp"
// #include "O_AsservXYRotateTest.hpp"
// #include "O_AsservTest.hpp"
#include "O_ButtonBarTest.hpp"
#include "O_IAbyPathTest.hpp"
#include "O_LcdBoardTest.hpp"
#include "O_LedBarTest.hpp"
#include "O_SensorsTest.hpp"
#include "O_ServoObjectsTest.hpp"
#include "O_ServoStepTest.hpp"
#include "O_TiretteTest.hpp"

#include "OPOS6UL_RobotExtended.hpp"
#include "utils/ConsoleManager.hpp"
#include "Robot.hpp"
#include "HardwareConfig.hpp"
#include "thread/Thread.hpp"
#include <sys/mman.h>
#include <cerrno>
#include <cstring>

using namespace std;

int main(int argc, char** argv)
{
    // Charger la config hardware depuis le repertoire de l'executable
    HardwareConfig::instance().load(argv[0]);

    // Verrouille toute la mémoire en RAM pour éviter les page faults (stalls 1-10ms)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        cerr << "mlockall FAILED: " << strerror(errno) << endl;
    }

    utils::set_realtime_priority(50, "Main"); //set priority MAX 99

    //Specific Robot BigPMX
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    // TODO: Re-enable test registrations once tests are migrated to PMX-CORTEX
    robot.getConsoleManager().add(new O_LedBarTest());
    robot.getConsoleManager().add(new O_TiretteTest());
    robot.getConsoleManager().add(new O_ButtonBarTest());
    robot.getConsoleManager().add(new O_LcdBoardTest());
    robot.getConsoleManager().add(new O_ActionManagerTimerTest());
    robot.getConsoleManager().add(new O_IAByPathTest());
    // robot.getConsoleManager().add(new O_AsservEsialTest());
    // robot.getConsoleManager().add(new O_AsservLineRotateTest());
    // robot.getConsoleManager().add(new O_AsservXYRotateTest());
    // robot.getConsoleManager().add(new O_AsservTest());
    // robot.getConsoleManager().add(new O_Asserv_SquareTest());
    // robot.getConsoleManager().add(new O_Asserv_CalageTest());

    robot.getConsoleManager().add(new O_ServoStepTest());
    robot.getConsoleManager().add(new O_ServoObjectsTest());
    robot.getConsoleManager().add(new O_SensorsTest());

    robot.parseConsoleArgs(argc, argv, false);

    //start the Robot (functional tests or match)
    robot.begin(argc, argv);


    return 0;
}
