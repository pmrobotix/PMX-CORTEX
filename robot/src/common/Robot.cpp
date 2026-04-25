/*!
 * \file
 * \brief Implémentation de la classe Robot.
 */

#include "Robot.hpp"

#include <stdio.h>
#include <stdlib.h>

#include "log/SvgWriter.hpp"
#include "geometry/TableGeometry.hpp"
#include "log/appender/TelemetryAppender.hpp"
//#include "Asserv/MotorControl.hpp" // not migrated yet
//#include "Asserv/MovingBase.hpp" // not migrated yet
#include "utils/ConsoleKeyInput.hpp"

#ifdef SIMU
#include <sys/ipc.h>
#include <sys/msg.h>
#endif
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "log/Logger.hpp"
#include "action/Actions.hpp"
#include "asserv/Asserv.hpp"

using namespace std;

Robot::Robot() :
        chrono_("Robot"), myColor_(PMXBLUE), cArgs_("", "(c) PM-ROBOTIX 2025", "/") // use character "/" instead of "-" for arguments
{
    points = 0;
    tabletest = false;

    actions_default_ = NULL;
    asserv_default_ = NULL;

    empty_ = 0;
    useExternalEncoder_ = 0;
    skipSetup_ = 0;
    end90s_ = 0;

    skipEndOfMatch = false;

    configureDefaultConsoleArgs();
    sharedPosition_ = ARobotPositionShared::create();
    tableGeometry_ = nullptr; // chaque robot concret l'instancie via setTableGeometry()
    sensors_ = nullptr;      // chaque robot concret l'instancie via setSensors()
}

Robot::~Robot() {
    // ORDRE IMPORTANT : arreter les threads producteurs AVANT de fermer le SVG.
    // Sinon le thread CBOR (AsservCborDriver) ou le scheduler peuvent ecrire
    // des <circle> APRES </svg> (SVG invalide).
    stopMotionTimerAndActionManager();
    svgPrintEndOfFile();
    delete tableGeometry_;
    //Stop le log s'il existe (core dump sinon)
    logs::LoggerFactory::instance().stopLog();
}

// Wrappers vers les transformations de couleur de l'Asserv.
float Robot::changeMatchX(float x_mm, float width) {
    return asserv_default_->changeMatchX(x_mm, width);
}
float Robot::changeMatchXMin(float x_mm, float width) {
    return asserv_default_->changeMatchXMin(x_mm, width);
}
float Robot::changeMatchAngleRad(float rad) {
    return asserv_default_->changeMatchAngleRad(rad);
}




//COLOR 0:GRIS / 1:ORANGE / 2:RED / 3:GREEN / 4:BLUE / 5:BLACK
void Robot::svgPrintPosition(int color) {

    if (asserv_default_ != NULL) {
        ROBOTPOSITION p = sharedPosition_->getRobotPosition();
        this->svgw().writePosition_Bot(p.x, p.y, p.theta, color);
    }
    else logger().error() << "asserv_default is NULL !" << logs::end;
}

void Robot::svgPrintEndOfFile() {
    // IMPORTANT : avant d'appeler cette methode, les threads producteurs (asserv
    // CBOR, scheduler des timers) doivent deja etre arretes par l'appelant.
    // Sinon des <circle> sont ecrits APRES </svg> -> SVG invalide.
    // Sequence type : freeMotion() + stopExtraActions() puis svgPrintEndOfFile().
    svg_->endHeader();
}

void Robot::configureDefaultConsoleArgs() {
#ifdef SIMU
    cArgs_.addOption('z', "Simulate button in a separate linux console, please execute this separately");
#endif
    // Add option "-h" with explanation...
    cArgs_.addOption('h', "Display usage help");

    cArgs_.addOption('k', "skip setup");

    // Convention PMX : color0=BLEU=primaire (coords strategie ecrites dans ce
    // repere). Defaut BLEU si aucune option passee. /y pour forcer JAUNE
    // (miroir x -> 3000-x). /b garde pour retro-compat et lisibilite (no-op).
    cArgs_.addOption('b', "color BLUE (defaut, explicite)");
    cArgs_.addOption('y', "color YELLOW (miroir strategie)");

    cArgs_.addArgument("type", "Type of match (t)est/(m)atch/(p)ause", "m");
    {
        Arguments::Option cOpt('n', ""); //TODO delete the /n, the t is enough
        cOpt.addArgument("num", "number of the functional test");
        cArgs_.addOption(cOpt);
    }

    {
        Arguments::Option cOpt('t', "");
        cOpt.addArgument("strategy", "name of the strategy of match", "all");
        cArgs_.addOption(cOpt);
    }

    {
        Arguments::Option cOpt('i', "Telemetry target IP");
        cOpt.addArgument("ip", "IP address of telemetry receiver", "192.168.3.101");
        cArgs_.addOption(cOpt);
    }

    {
        Arguments::Option cOpt('p', "Telemetry target UDP port");
        cOpt.addArgument("port", "UDP port of telemetry receiver", "9870");
        cArgs_.addOption(cOpt);
    }

    {
        // Note: le path ne peut PAS commencer par / (c'est un marqueur d'option).
        // Utilisez un path relatif ou un nom simple. Le defaut pointe vers le
        // simulateur du repo, depuis build-*/bin/.
        Arguments::Option cOpt('e', "Export playground + IA zones to simulator JSON");
        cOpt.addArgument("path", "Path to table.json (no leading /, relative to cwd)",
                         "../../../simulator/resources/2026/table.json");
        cArgs_.addOption(cOpt);
    }

    cArgs_.addOption('d', "Dry-run: after export zones, exit without starting the match");

    {
        // /s <name>  -> charge strategy<name>.json (a cote de l'exe).
        // Ex: /s PMX0  -> strategyPMX0.json
        Arguments::Option cOpt('s', "Run JSON strategy runner (fallback hardcode si absent)");
        cOpt.addArgument("name", "Strategy name (ex: 'PMX0' -> strategyPMX0.json)", "");
        cArgs_.addOption(cOpt);
    }
}

void Robot::loadInitJsonForCurrentStrategy()
{
    // Reset aux defauts si pas de strat JSON courante (mode legacy "all").
    if (strategyJsonName_.empty()) {
        initPoseX_         = 300.0f;
        initPoseY_         = 130.0f;
        initPoseThetaDeg_  = 90.0f;
        setposTasks_.clear();
        return;
    }
    std::string iPath = initJsonPath();
    FILE* fi = std::fopen(iPath.c_str(), "r");
    if (!fi) {
        logger().warn() << "loadInitJsonForCurrentStrategy: init JSON not found: " << iPath
                        << " (cwd-relative) - garde valeurs courantes" << logs::end;
        return;
    }
    std::fclose(fi);
    InitData initData;
    if (!parseInitFromFile(iPath, initData)) {
        logger().warn() << "loadInitJsonForCurrentStrategy: parse failed for " << iPath
                        << " - garde valeurs courantes" << logs::end;
        return;
    }
    initPoseX_         = initData.x;
    initPoseY_         = initData.y;
    initPoseThetaDeg_  = initData.thetaDeg;
    setposTasks_       = std::move(initData.setposTasks);
}

void Robot::parseConsoleArgs(int argc, char** argv, bool stopWithErrors) {
    if (!cArgs_.parse(argc, argv, stopWithErrors)) {
        logger().debug() << "Error parsing DEFAULT" << logs::end;
        sleep(1);
        exit(-1);
    }

    if (cArgs_['h']) {
        std::cout << "Available functional tests: " << std::endl;
        cmanager_.displayAvailableTests("", -1);
        cArgs_.usage();
        exit(0);
    }

    // Export zones JSON (pour simulateur)
    if (cArgs_['e']) {
        exportZonesPath_ = cArgs_['e']["path"];
    }
    if (cArgs_['d']) {
        exportZonesDryRun_ = true;
    }

    // Strategy JSON runner + Init JSON. /s <name> est l'override CLI : il fixe
    // strategyJsonName_ ET strategy_ (le menu peut encore les changer ensuite).
    // Sans /s : defaut "PMX1" (cf. constructeur Robot).
    if (cArgs_['s']) {
        strategyJsonName_ = cArgs_['s']["name"];
        strategy_ = strategyJsonName_;  // sync avec strategy_ pour cohérence menu/JSON
    }
    // Fail-fast (avant toute init hardware) : valider que strategy<name>.json et
    // init<name>.json existent et sont parsables. Le defaut PMX1 doit donc avoir
    // ses fichiers presents dans cwd.
    std::string sPath = strategyJsonPath();
    if (!sPath.empty()) {
        FILE* f = std::fopen(sPath.c_str(), "r");
        if (!f) {
            std::cerr << "ERROR: strategy JSON file not found: " << sPath
                      << " (cwd-relative). Aborting." << std::endl;
            std::exit(1);
        }
        std::fclose(f);

        std::string iPath = initJsonPath();
        FILE* fi = std::fopen(iPath.c_str(), "r");
        if (!fi) {
            std::cerr << "ERROR: init JSON file not found: " << iPath
                      << " (cwd-relative). Aborting." << std::endl;
            std::exit(1);
        }
        std::fclose(fi);

        InitData initData;
        if (!parseInitFromFile(iPath, initData)) {
            std::cerr << "ERROR: parse failed for " << iPath << ". Aborting." << std::endl;
            std::exit(1);
        }
        initPoseX_ = initData.x;
        initPoseY_ = initData.y;
        initPoseThetaDeg_ = initData.thetaDeg;
        setposTasks_ = std::move(initData.setposTasks);
    }

    // Reconfigure telemetry appender with command line args
    if (cArgs_['i'] || cArgs_['p']) {
        std::string ip = cArgs_['i'] ? cArgs_['i']["ip"] : "192.168.3.101";
        int port = cArgs_['p'] ? std::atoi(cArgs_['p']["port"].c_str()) : 9870;
        logs::Appender *app = logs::LoggerFactory::instance().appender("net");
        if (app != nullptr) {
            auto *telemetry = dynamic_cast<logs::TelemetryAppender *>(app);
            if (telemetry != nullptr) {
                telemetry->configure(ip, port);
                logger().info() << "Telemetry configured: " << ip << ":" << port << logs::end;
            }
        }
    }
}



void Robot::begin(int argc, char** argv) {
    int num = -1;
    string select = "-";
    string color = "-";
    string strat = "-";

#ifdef SIMU //cas de la simulation sous linux
    //http://jean-luc.massat.perso.luminy.univ-amu.fr/ens/docs/IPC.html
    //only for SIMU to simulate a non blocking getch() in a separate window console with /z
    if (cArgs_['z']) {
        int res;
        int frequete;
        printf("Send button from keyboard : BACK ENTER UP DOWN LEFT RIGHT\n");
        frequete = msgget(CLEF_REQUETES, 0700 | IPC_CREAT);
        if (frequete == -1) {
            perror("msgget");
            sleep(1);
            exit(0);
        }
        while (1) {
            char cInput;
            cInput = ConsoleKeyInput::mygetch(); //wait a user action
            //printf("button= %d<\n", cInput);
            if (cInput == 27)            // if ch is the escape sequence with num code 27, k turns 1 to signal the next
                    {
                cInput = ConsoleKeyInput::mygetch();
                if (cInput == 91) // if the previous char was 27, and the current 91, k turns 2 for further use
                        {
                    cInput = ConsoleKeyInput::mygetch();
                }
            }

            printf("final button= %d \n", cInput);

            switch (cInput) {
                case 10:
                    strcpy(msg_ipc.mtext, "enter");
                    break;
                case 127:
                    strcpy(msg_ipc.mtext, "back");
                    break;
                case 65:
                    strcpy(msg_ipc.mtext, "up");
                    break;
                case 66:
                    strcpy(msg_ipc.mtext, "down");
                    break;
                case 67:
                    strcpy(msg_ipc.mtext, "right");
                    break;
                case 68:
                    strcpy(msg_ipc.mtext, "left");
                    break;

                default:

                    break;
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));

            }

            msg_ipc.mtype = getpid();

            res = msgsnd(frequete, &msg_ipc, strlen(msg_ipc.mtext) + 1, 0);
            if (res == -1) {
                perror("msgsnd");
                exit(0);
            }
            std::this_thread::yield();
        }
    }
#endif

    if (cArgs_["type"] == "p" || cArgs_["type"] == "P") {
        //display all functional tests
        cmanager_.displayAvailableTests("", -1);

        if (!cArgs_['k']) {
            //------------- Pour debug
            //pause s'il n'y a pas tous les elements pour visualiser le log d'erreur
            char cInput;
            Robot::logger().info() << "Press Enter key to continue ..." << logs::end;

            do {
                cInput = ConsoleKeyInput::mygetch();
                switch (cInput) {

                    case 10:
                        //printf("Enter key!\n");
                        break;
                    case 127:
                        //printf("Back key!\n");
                        cout << "Exit !\n" << endl;
                        //cout << default_console << endl;
                        sleep(1);
                        exit(0);
                        break;
                }
                utils::sleep_for_micros(1000);
                std::this_thread::yield();
            } while (cInput != 10);
            //---------------fin Pour debug
        }
    }
    //logger().debug() << "" << logs::end;

    //logger().debug() << "type = " << cArgs_["type"] << logs::end;

    //logger().debug() << "Option c set " << (int) cArgs_['c'] << ", color = " << " " << cArgs_['c']["color"] << logs::end;

//    if (cArgs_['c']) {
//        color = cArgs_['c']["color"];
//        if (color == "violet" || color == "v") this->setMyColor(PMXBLUE);
//        else if (color == "yellow" || color == "jaune" || color == "j" || color == "y") this->setMyColor(PMXBLUE);
//        else {
//            this->setMyColor(PMXNOCOLOR);
//            logger().error() << "setMyColor(NOCOLOR)" << logs::end;
//            exit(-1);
//        }
//        logger().debug() << "setMyColor DONE : " << this->getMyColor() << logs::end;
//    }
//    else {
//        //defaut si aucune couleur n'est specifiee
//        this->setMyColor(PMXBLUE);
//    }

    // /y = JAUNE explicite ; /b = BLEU explicite ; sinon defaut BLEU (color0 primaire)
    if (cArgs_['y']) {
        this->setMyColor(PMXYELLOW);
    }
    else {
        this->setMyColor(PMXBLUE);
    }
    logger().debug() << "setMyColor done; getMyColor() = " << getMyColor() << logs::end;




    // /s <name> a deja ete traite dans parseConsoleArgs() (fixe strategyJsonName_
    // ET strategy_). Sinon, on conserve les defauts (PMX1) du constructeur Robot.
    strat = strategy_;
    if (cArgs_['s']) {
        logger().info() << "strategy selected (via /s) = " << strat << logs::end;
    } else {
        logger().info() << "strategy default = " << strat << logs::end;
    }

    //test number
    if (cArgs_['n']) {
        num = atoi(cArgs_['n']["num"].c_str());
        logger().debug() << "Option n set " << (int) cArgs_['n'] << ", num = " << num << logs::end;
    }

    //skip state
    if (cArgs_['k']) {
        logger().debug() << "skip = " << (int) cArgs_['k'] << logs::end;
        this->skipSetup(true);
    }
    else this->skipSetup(false);

    // Chercher si type correspond a un code mnemonique de test (ex: "lr", "sq")
    if (cArgs_["type"] != "m" && cArgs_["type"] != "M"
        && cArgs_["type"] != "t" && cArgs_["type"] != "T"
        && cArgs_["type"] != "p" && cArgs_["type"] != "P") {
        int codeNum = cmanager_.findByCode(cArgs_["type"]);
        if (codeNum > 0) {
            cArgs_["type"] = "t";
            num = codeNum;
        }
    }

    if (cArgs_["type"] != "m" && cArgs_["type"] != "t" && cArgs_["type"] != "T" && cArgs_["type"] != "M") {
        select = cmanager_.displayMenuFirstArgu();
        if (select == "-") {
            logger().error() << "displayMenuFirstArgu bad return " << logs::end;
            sleep(1);
            exit(-1);
        }
        cArgs_["type"] = select;
    }

    //functional test cases
    if (cArgs_["type"] == "t" or cArgs_["type"] == "T") {
        if (num > 0) {
            //execute defined test
            cmanager_.run(num, argc, argv);
        }
        else {
            //ask param
            cmanager_.displayMenuFunctionalTestsAndRun(argc, argv);
        }
    }
}

void Robot::stopMotionTimerAndActionManager() {

    if (passerv() != NULL) {
        this->passerv()->stopMotionTimerAndOdo();
    }
    else logger().error() << "asserv_default is NULL ! " << logs::end;

    if (actions_default_ != NULL) {

        this->actions_default_->clearAll(); //clear actions and timers
        this->actions_default_->waitAndStopManagers();
    }
    else logger().error() << "actions_default is NULL ! " << logs::end;
}

void Robot::freeMotion() {
    this->passerv()->freeMotion();
    //this->passerv()->stopMotors();
}

void Robot::resetDisplayTS() {
    logger().error() << "resetDisplayTS ! (To be surcharged) " << logs::end;
}

void Robot::displayTS(TRAJ_STATE ts) {
    logger().error() << "displayTS ! (To be surcharged) " << logs::end;
}
void Robot::resetDisplayObstacle() {
    logger().error() << "resetDisplayObstacle ! (To be surcharged) " << logs::end;
}

void Robot::displayObstacle(int level) {
    logger().error() << "displayObstacle ! (To be surcharged) " << logs::end;
}
