
#include "O_ActionManagerTimerTest.hpp"
#include "O_Asserv_CalageTest.hpp"
#include "O_Asserv_SquareTest.hpp"
#include "O_AsservCalibrationTest.hpp"
#include "O_AsservLineRotateTest.hpp"
#include "O_AsservXYRotateTest.hpp"
#include "O_AsservTest.hpp"
#include "O_GroveColorTest.hpp"
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
#include <csignal>
#include <cstring>

using namespace std;

// Handler SIGINT (Ctrl+C) : ferme proprement les fichiers SVG, flush les logs, puis quitte.
static void sigintHandler(int)
{
    // 1) Écrit </g></svg> dans le buffer du logger SVG
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    robot.svgPrintEndOfFile();
    // 2) Flush les buffers mémoire vers les fichiers (SvgAppender) et ferme les logs
    logs::LoggerFactory::instance().stopLog();
    // 3) exit() appelle les destructeurs et flush les streams (contrairement à _exit)
    exit(0);
}

int main(int argc, char** argv)
{
    // Charger la config hardware depuis le repertoire de l'executable
    HardwareConfig::instance().load(argv[0]);

    // Verrouille toute la mémoire en RAM pour éviter les page faults (stalls 1-10ms)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        cerr << "mlockall FAILED: " << strerror(errno) << endl;
    }

    utils::set_realtime_priority(50, "Main"); //set priority MAX 99

    // Capture Ctrl+C pour fermer proprement les SVG
    signal(SIGINT, sigintHandler);

    //Specific Robot BigPMX
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();

    robot.getConsoleManager().add(new O_LedBarTest());
    robot.getConsoleManager().add(new O_TiretteTest());
    robot.getConsoleManager().add(new O_ButtonBarTest());
    robot.getConsoleManager().add(new O_LcdBoardTest());
    robot.getConsoleManager().add(new O_ActionManagerTimerTest());
    robot.getConsoleManager().add(new O_IAByPathTest());
    
    robot.getConsoleManager().add(new O_AsservCalibrationTest());
    robot.getConsoleManager().add(new O_AsservLineRotateTest());
    robot.getConsoleManager().add(new O_AsservXYRotateTest());
    robot.getConsoleManager().add(new O_AsservTest());
    robot.getConsoleManager().add(new O_Asserv_SquareTest());
    robot.getConsoleManager().add(new O_Asserv_CalageTest());
    robot.getConsoleManager().add(new O_GroveColorTest());

    robot.getConsoleManager().add(new O_ServoStepTest());
    robot.getConsoleManager().add(new O_ServoObjectsTest());
    robot.getConsoleManager().add(new O_SensorsTest());

    robot.parseConsoleArgs(argc, argv, false);

    //start the Robot (functional tests or match)
    robot.begin(argc, argv);


    return 0;
}
