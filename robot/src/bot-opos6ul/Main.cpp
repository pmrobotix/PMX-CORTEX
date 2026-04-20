
#include "O_ActionTimerSchedulerTest.hpp"
#include "O_Asserv_CalageTest.hpp"
#include "O_Asserv_SquareTest.hpp"
#include "O_AsservCalibrationTest.hpp"
#include "O_AsservLineRotateTest.hpp"
#include "O_AsservLineRotateOldTest.hpp"
#include "O_AsservWaypointTest.hpp"
#include "O_NavigatorMovementTest.hpp"
#include "O_NavigatorBackTest.hpp"
#include "O_DetectionForwardTest.hpp"
#include "O_DetectionBackwardTest.hpp"
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
    OPOS6UL_RobotExtended &robot = OPOS6UL_RobotExtended::instance();
    // 1) Arreter les threads producteurs SVG (asserv CBOR + scheduler timers)
    //    pour qu'aucun <circle> ne soit ecrit apres </svg>
    robot.stopExtraActions();
    // 2) Écrit </g></svg> dans le buffer du logger SVG
    robot.svgPrintEndOfFile();
    // 3) Flush les buffers mémoire vers les fichiers (SvgAppender) et ferme les logs
    logs::LoggerFactory::instance().stopLog();
    // 4) exit() appelle les destructeurs et flush les streams (contrairement à _exit)
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

    robot.getConsoleManager().add(new O_LedBarTest());          //  1. led
    robot.getConsoleManager().add(new O_TiretteTest());         //  2. tir
    robot.getConsoleManager().add(new O_ButtonBarTest());       //  3. btn
    robot.getConsoleManager().add(new O_LcdBoardTest());        //  4. lcd
    robot.getConsoleManager().add(new O_ActionTimerSchedulerTest()); //  5. ats
    robot.getConsoleManager().add(new O_SensorsTest());         //  6. sns
    robot.getConsoleManager().add(new O_ServoStepTest());       //  7. ss
    robot.getConsoleManager().add(new O_ServoObjectsTest());    //  8. so

    robot.getConsoleManager().add(new O_IAByPathTest());        //  9. ia
    robot.getConsoleManager().add(new O_AsservCalibrationTest()); // 10. cal
    robot.getConsoleManager().add(new O_AsservLineRotateTest()); // 11. lr
    robot.getConsoleManager().add(new O_AsservWaypointTest());  // 12. wp
    robot.getConsoleManager().add(new O_NavigatorMovementTest()); // 13. nav
    robot.getConsoleManager().add(new O_NavigatorBackTest());   // 14. nb
    robot.getConsoleManager().add(new O_DetectionForwardTest());  //     detf (forward)
    robot.getConsoleManager().add(new O_DetectionBackwardTest()); //     detb (backward)
    //robot.getConsoleManager().add(new O_AsservLineRotateOldTest()); // deprecated
    robot.getConsoleManager().add(new O_AsservXYRotateTest());  // 15. xy
    robot.getConsoleManager().add(new O_AsservTest());          // 16. go
    robot.getConsoleManager().add(new O_Asserv_SquareTest());   // 17. sq
    robot.getConsoleManager().add(new O_Asserv_CalageTest());   // 18. rca
    robot.getConsoleManager().add(new O_GroveColorTest());      // 19. col

    robot.parseConsoleArgs(argc, argv, false);

    //start the Robot (functional tests or match)
    robot.begin(argc, argv);


    return 0;
}
