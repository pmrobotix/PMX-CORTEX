cmake_minimum_required(VERSION 3.21)
project(PMX-CORTEX LANGUAGES CXX C)

# --- Standard C++ ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Répertoires de sortie ---
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# --- Options communes ---
add_compile_options(-Wall -pthread)

# --- Optimisation Release ---
# SIMU : -O2 (perfs réalistes pour la simulation)
# ARM  : -Os (optimisation taille, comme dans PMX — mieux pour le cache L1 du Cortex-A7)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG" CACHE STRING "" FORCE)

# --- Plateforme : ARM ou SIMU ---
if(NOT TARGET_PLATFORM)
    set(TARGET_PLATFORM "SIMU")
endif()

message(STATUS "=== Plateforme cible : ${TARGET_PLATFORM} ===")

if(TARGET_PLATFORM STREQUAL "SIMU")
    add_compile_definitions(SIMU)
    message(STATUS "Mode SIMU - compilation native")
elseif(TARGET_PLATFORM STREQUAL "ARM")
    set(CMAKE_CXX_FLAGS_RELEASE "-Os -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG" CACHE STRING "" FORCE)
    add_compile_options(-Wno-psabi)  # masquer les notes ABI GCC 7.1 sur ARM
    set(AS_DEVICES_INCLUDE
        "/install/opos6ul-git/buildroot/output/host/arm-buildroot-linux-gnueabihf/sysroot/usr/include/as_devices")
    include_directories(${AS_DEVICES_INCLUDE})
    message(STATUS "Mode ARM - cross-compilation OPOS6UL (-Os)")
endif()

# =============================================
# Bibliothèques externes (git submodules)
# =============================================
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/simple-svg
    ${CMAKE_CURRENT_BINARY_DIR}/libs/simple-svg)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/PathFinding
    ${CMAKE_CURRENT_BINARY_DIR}/libs/PathFinding)

# =============================================
# Bibliothèque : pmx-common (briques de base)
# =============================================
add_library(pmx-common STATIC
    # thread
    src/common/thread/Thread.cpp
    src/common/thread/Mutex.cpp
    # utils
    src/common/utils/Chronometer.cpp
    src/common/utils/itoa.cpp
    # log
    src/common/log/Level.cpp
    src/common/log/Logger.cpp
    src/common/log/LoggerFactory.cpp
    src/common/log/SvgWriter.cpp
    src/common/log/backward.cpp
    src/common/log/appender/MemoryAppender.cpp
    src/common/log/appender/ConsoleAppender.cpp
    src/common/log/appender/FileAppender.cpp
    src/common/log/appender/SvgAppender.cpp
    src/common/log/appender/TelemetryAppender.cpp
    # timer
    src/common/timer/ActionTimerScheduler.cpp
    # state
    src/common/state/Automate.cpp
    # asserv (generique)
    src/common/asserv/Asserv.cpp
    # asserv.esial (implementation EsialRobotik)
    src/common/asserv.esial/AsservEsialR.cpp
    src/common/asserv.esial/codeurs/BotCodeurs.cpp
    src/common/asserv.esial/codeurs/Codeur.cpp
    src/common/asserv.esial/codeurs/CodeursDirects.cpp
    src/common/asserv.esial/commandManager/CMDList/CMDList.cpp
    src/common/asserv.esial/commandManager/CommandManagerA.cpp
    src/common/asserv.esial/config/config.cpp
    src/common/asserv.esial/consignController/ConsignController.cpp
    src/common/asserv.esial/filtres/Pid/Pid.cpp
    src/common/asserv.esial/filtres/QuadRampDerivee/QuadRampDerivee.cpp
    src/common/asserv.esial/motorsController/BotMotors.cpp
    src/common/asserv.esial/odometrie/Odometrie.cpp
    src/common/asserv.esial/regulateur/Regulateur.cpp
    src/common/asserv.esial/Utils/Utils.cpp
    # ia
    src/common/ia/IA.cpp
    src/common/ia/IAbyPath.cpp
    src/common/ia/IAbyZone.cpp
    src/common/ia/ZoneJsonExporter.cpp
    src/common/ia/StrategyJsonParser.cpp
    src/common/ia/StrategyJsonRunner.cpp
    src/common/ia/ActionRegistry.cpp
    src/common/ia/FlagManager.cpp
    # navigator
    src/common/navigator/Navigator.cpp
    # utils
    src/common/utils/Arguments.cpp
    src/common/utils/ConsoleManager.cpp
    # robot
    src/common/Robot.cpp
    # action (wrappers)
    src/common/action/Actions.cpp
    src/common/action/ButtonBar.cpp
    src/common/action/LcdShield.cpp
    src/common/action/LedBar.cpp
    src/common/action/ServoObjectsSystem.cpp
    src/common/action/ServoUsingMotor.cpp
    src/common/action/SoundBar.cpp
    src/common/action/Tirette.cpp
    src/common/action/Sensors.cpp
    src/common/geometry/ObstacleZone.cpp
    # menu (refactor O_State_NewInit - voir robot/md/O_STATE_NEW_INIT.md)
    src/common/menu/MenuController.cpp
    src/common/menu/MenuShieldLCD.cpp
    src/common/menu/MenuBeaconLCDTouch.cpp
    # hardware config
    src/common/HardwareConfig.cpp
)

target_include_directories(pmx-common PUBLIC
    src/common
    src/common/asserv.esial
)

target_link_libraries(pmx-common PUBLIC pthread rt simple-svg pmr-pathfinding)

# =============================================
# Bibliothèque driver-stub : stubs simu toujours disponibles
# =============================================
add_library(pmx-driver-stub STATIC
    src/driver-simu/LedDriver.cpp
    src/driver-simu/LcdShieldDriver.cpp
    src/driver-simu/ButtonDriver.cpp
    src/driver-simu/SwitchDriver.cpp
    src/driver-simu/SensorsDriver.cpp
    src/driver-simu/ServoDriver.cpp
    src/driver-simu/AsservDriver.cpp
    src/driver-simu/ColorDriver.cpp
)

target_include_directories(pmx-driver-stub PUBLIC
    src/driver-simu
)

target_link_libraries(pmx-driver-stub PUBLIC pmx-common)

# =============================================
# Bibliothèque driver : sélection automatique
# =============================================
if(TARGET_PLATFORM STREQUAL "ARM")
    # --- pmx-driver-arm (cross-compilation OPOS6UL) ---
    add_library(pmx-driver-arm STATIC
        src/driver-arm/LedDriver.cpp
        src/driver-arm/SwitchDriver.cpp
        src/driver-arm/GpioPCA9555.cpp
        src/driver-arm/ButtonDriver.cpp
        src/driver-arm/SoundDriver.cpp
        src/driver-arm/LcdShieldDriver.cpp
        src/driver-arm/RobotPositionShared.cpp
        src/driver-arm/Adafruit_MCP23017.cpp
        src/driver-arm/Adafruit_RGBLCDShield.cpp
        src/driver-arm/BeaconSensors.cpp
        src/driver-arm/CCAx12Teensy.cpp
        src/driver-arm/Gp2y0e02b.cpp
        src/driver-arm/GroveColorSensor.cpp
        src/driver-arm/SensorsDriver.cpp
        src/driver-arm/serialib.cpp
        src/driver-arm/ServoDriver.cpp
        src/driver-arm/MD25.cpp
        src/driver-arm/ServoUsingMotorDriver.cpp
        src/driver-arm/ActionDriver.cpp
        src/driver-arm/AsservCborDriver.cpp
        src/driver-arm/cbor/CborFrameDecoder.cpp
    )

    # qcbor library (CBOR encode/decode, C language)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/qcbor
        ${CMAKE_CURRENT_BINARY_DIR}/libs/qcbor)

    target_include_directories(pmx-driver-arm PUBLIC
        src/driver-arm
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/crc
    )

    target_link_libraries(pmx-driver-arm PUBLIC pmx-common pmx-driver-stub as_devices as_devices_cpp qcbor)

    set(DRIVER_LIB pmx-driver-arm)
else()
    # --- pmx-driver-simu (compilation native PC) ---
    add_library(pmx-driver-simu STATIC
        src/driver-simu/DriverFactories.cpp
        src/driver-simu/SoundDriver.cpp
        src/driver-simu/RobotPositionShared.cpp
        src/driver-simu/ServoUsingMotorDriver.cpp
        src/driver-simu/ActionDriver.cpp
    )

    target_include_directories(pmx-driver-simu PUBLIC
        src/driver-simu
    )

    target_link_libraries(pmx-driver-simu PUBLIC pmx-common pmx-driver-stub)

    set(DRIVER_LIB pmx-driver-simu)
endif()

# =============================================
# Cible principale : bot-opos6ul (executable)
# =============================================
if(DRIVER_LIB)
    add_executable(bot-opos6ul
        src/bot-opos6ul/Main.cpp
        src/bot-opos6ul/LoggerInitialize.cpp
        src/bot-opos6ul/OPOS6UL_RobotExtended.cpp
        src/bot-opos6ul/OPOS6UL_ActionsExtended.cpp
        src/bot-opos6ul/OPOS6UL_AsservExtended.cpp
        src/bot-opos6ul/OPOS6UL_IAExtended.cpp
        src/bot-opos6ul/OPOS6UL_SvgWriterExtended.cpp
        src/bot-opos6ul/O_State_DecisionMakerIA.cpp
        src/bot-opos6ul/StrategyActions2026.cpp
        src/bot-opos6ul/O_State_NewInit.cpp
        src/bot-opos6ul/O_State_WaitEndOfMatch.cpp
        src/bot-test-opos6ul/O_LedBarTest.cpp
        src/bot-test-opos6ul/O_TiretteTest.cpp
        src/bot-test-opos6ul/O_ButtonBarTest.cpp
        src/bot-test-opos6ul/O_LcdBoardTest.cpp
        src/bot-test-opos6ul/O_ActionTimerSchedulerTest.cpp
        src/bot-test-opos6ul/O_IAbyPathTest.cpp
        src/bot-test-opos6ul/O_AsservLineRotateTest.cpp
        src/bot-test-opos6ul/O_AsservWaypointTest.cpp
        src/bot-test-opos6ul/O_NavigatorMovementTest.cpp
        src/bot-test-opos6ul/O_NavigatorBackTest.cpp
        src/bot-test-opos6ul/O_DetectionScenarioTest.cpp
        src/bot-test-opos6ul/O_DetectionForwardTest.cpp
        src/bot-test-opos6ul/O_DetectionBackwardTest.cpp
        src/bot-test-opos6ul/O_DetectionRetryForwardTest.cpp
        src/bot-test-opos6ul/O_DetectionRetryBackwardTest.cpp
        src/bot-test-opos6ul/O_StrategyJsonRunnerTest.cpp
        src/bot-test-opos6ul/O_AsservXYRotateTest.cpp
        src/bot-test-opos6ul/O_AsservTest.cpp
        src/bot-test-opos6ul/O_AsservCalibrationTest.cpp
        src/bot-test-opos6ul/O_Asserv_SquareTest.cpp
        src/bot-test-opos6ul/O_Asserv_CalageTest.cpp
        src/bot-test-opos6ul/O_GroveColorTest.cpp
        src/bot-test-opos6ul/O_ServoStepTest.cpp
        src/bot-test-opos6ul/O_ServoObjectsTest.cpp
        src/bot-test-opos6ul/O_SensorsTest.cpp
    )

    target_include_directories(bot-opos6ul PRIVATE
        src/bot-opos6ul
        src/bot-test-opos6ul
    )

    target_link_libraries(bot-opos6ul PRIVATE pmx-common ${DRIVER_LIB} pmr-pathfinding)

    # =============================================
    # Copie automatique des fichiers runtime dans bin/
    #
    # Utilise add_custom_command(OUTPUT ... DEPENDS ...) au lieu de POST_BUILD :
    # CMake suit les changements des fichiers SOURCES (.json/.conf/...) et
    # re-declenche la copie quand un source est modifie, meme si bot-opos6ul
    # lui-meme n'a pas ete recompile.
    # =============================================

    # Liste des paires (source, destination) a copier
    # bot-opos6ul/<file> -> $<TARGET_FILE_DIR:bot-opos6ul>/<file>
    set(BOT_OPOS6UL_RUNTIME_FILES
        hardware.conf
        config_OPOS6UL_Robot.txt
        initPMX0.json   strategyPMX0.json
        initPMX1.json   strategyPMX1.json
        initPMX2.json   strategyPMX2.json
        initPMX3.json   strategyPMX3.json
        initTEST0.json  strategyTEST0.json
        initTEST1.json  strategyTEST1.json
        initTEST2.json  strategyTEST2.json
        initTEST3.json  strategyTEST3.json
        initTEST4.json  strategyTEST4.json
        initTEST5.json  strategyTEST5.json
        initTEST6.json  strategyTEST6.json
    )

    set(BOT_OPOS6UL_RUNTIME_OUTPUTS "")
    foreach(f ${BOT_OPOS6UL_RUNTIME_FILES})
        set(SRC ${CMAKE_SOURCE_DIR}/src/bot-opos6ul/${f})
        set(DST ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${f})
        add_custom_command(
            OUTPUT ${DST}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SRC} ${DST}
            DEPENDS ${SRC}
            COMMENT "Copy runtime ${f}"
        )
        list(APPEND BOT_OPOS6UL_RUNTIME_OUTPUTS ${DST})
    endforeach()

    # Ressources SVG (index.html + table.png depuis common/log/resources-svg/)
    foreach(pair
        "common/log/resources-svg/index.html|index.html"
        "common/log/resources-svg/table.png|table.png"
    )
        string(REPLACE "|" ";" pair_list ${pair})
        list(GET pair_list 0 src_rel)
        list(GET pair_list 1 dst_name)
        set(SRC ${CMAKE_SOURCE_DIR}/src/${src_rel})
        set(DST ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${dst_name})
        add_custom_command(
            OUTPUT ${DST}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SRC} ${DST}
            DEPENDS ${SRC}
            COMMENT "Copy runtime ${dst_name}"
        )
        list(APPEND BOT_OPOS6UL_RUNTIME_OUTPUTS ${DST})
    endforeach()

    # Scenarios JSON pour O_StrategyJsonRunnerTest (jrun) — copie de tout le dossier.
    # Pas de DEPENDS individuel possible (glob), on utilise un timestamp file.
    set(SCENARIOS_STAMP ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/.scenarios_copied)
    file(GLOB SCENARIOS_SRC ${CMAKE_SOURCE_DIR}/src/bot-test-opos6ul/scenarios/*)
    add_custom_command(
        OUTPUT ${SCENARIOS_STAMP}
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/src/bot-test-opos6ul/scenarios
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_COMMAND} -E touch ${SCENARIOS_STAMP}
        DEPENDS ${SCENARIOS_SRC}
        COMMENT "Copy scenarios JSON (jrun)"
    )
    list(APPEND BOT_OPOS6UL_RUNTIME_OUTPUTS ${SCENARIOS_STAMP})

    # Target qui rebundle toutes les copies. ALL = inclus dans le build par defaut.
    # add_dependencies pour que make bot-opos6ul declenche aussi les copies.
    add_custom_target(bot-opos6ul-runtime ALL DEPENDS ${BOT_OPOS6UL_RUNTIME_OUTPUTS})
    add_dependencies(bot-opos6ul bot-opos6ul-runtime)
endif()

# =============================================
# Bibliothèque : pmx-suite (framework de test)
# =============================================
add_library(pmx-suite STATIC
    test/suite/UnitTest.cpp
    test/suite/UnitTestSuite.cpp
    test/suite/UnitTestAppender.cpp
)

target_include_directories(pmx-suite PUBLIC
    test/suite
)

target_link_libraries(pmx-suite PUBLIC pmx-common)

# =============================================
# Cible test : common-test
# =============================================
add_executable(common-test
    test/common/Main.cpp
    test/common/LoggerInitialize.cpp
    test/common/MutexTest.cpp
    test/common/ThreadTest.cpp
    test/common/ChronometerTest.cpp
    test/common/LevelTest.cpp
    test/common/LoggerTest.cpp
    test/common/ExceptionTest.cpp
    test/common/PointerListTest.cpp
    test/common/FileAppenderTest.cpp
    test/common/SvgAppenderTest.cpp
    test/common/ActionTimerSchedulerTest.cpp
    test/common/RetryPolicyTest.cpp
    test/common/TableGeometryTest.cpp
    test/common/ObstacleZoneTest.cpp
    test/common/DetectionEventTest.cpp
    test/common/IsOnPathTest.cpp
    test/common/FlagManagerTest.cpp
    test/common/StrategyJsonParserTest.cpp
)

target_link_libraries(common-test PRIVATE pmx-suite)

# =============================================
# Cible test : driver-test
# =============================================
if(DRIVER_LIB)
    add_executable(driver-test
        test/driver/Main.cpp
        test/driver/LoggerInitialize.cpp
        test/driver/LedDriverTest.cpp
        test/driver/SwitchDriverTest.cpp
        test/driver/NavigatorTest.cpp
        test/driver/SensorsDriverTest.cpp
        # SensorsTest supprime : logique pure → ObstacleZoneTest (common-test), integration → O_SensorsTest
    )

    target_link_libraries(driver-test PRIVATE pmx-suite ${DRIVER_LIB} pmx-common ${DRIVER_LIB})

    # =============================================
    # Cible test : manual-test (verification visuelle/physique)
    # =============================================
    add_executable(manual-test
        test/manual/Main.cpp
        test/manual/LoggerInitialize.cpp
        test/manual/LedManualTest.cpp
        test/manual/SwitchManualTest.cpp
        test/manual/AsservDriverManualTest.cpp
        test/manual/ButtonDriverManualTest.cpp
        test/manual/LcdShieldDriverManualTest.cpp
        test/manual/ServoDriverManualTest.cpp
        test/manual/ColorDriverManualTest.cpp
        test/manual/SensorDriverManualTest.cpp
        test/manual/ScreenFramebufferManualTest.cpp
    )

    target_link_libraries(manual-test PRIVATE pmx-suite ${DRIVER_LIB})

    # =============================================
    # Cible test : bench (metriques de performance)
    # =============================================
    add_executable(bench
        test/bench/Main.cpp
        test/bench/LoggerInitialize.cpp
        test/bench/ActionTimerSchedulerBench.cpp
        test/bench/TimerPosix.cpp
        test/bench/TimerPosixStandaloneBench.cpp
    )

    target_link_libraries(bench PRIVATE pmx-suite)
endif()

# =============================================
# CTest — lancement des tests depuis le repertoire bin/
# pour que les SVG et fichiers de config soient au bon endroit
# =============================================
enable_testing()

add_test(NAME common-test
    COMMAND $<TARGET_FILE:common-test>
    WORKING_DIRECTORY $<TARGET_FILE_DIR:common-test>)

if(DRIVER_LIB)
    add_test(NAME driver-test
        COMMAND $<TARGET_FILE:driver-test>
        WORKING_DIRECTORY $<TARGET_FILE_DIR:driver-test>)

    add_test(NAME bot-opos6ul-nav-test
        COMMAND $<TARGET_FILE:bot-opos6ul> t /n 10
        WORKING_DIRECTORY $<TARGET_FILE_DIR:bot-opos6ul>)
    set_tests_properties(bot-opos6ul-nav-test PROPERTIES TIMEOUT 120)
endif()
