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
    src/common/timer/ActionManagerTimer.cpp
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
        src/bot-opos6ul/O_State_Init.cpp
        src/bot-opos6ul/O_State_WaitEndOfMatch.cpp
        src/bot-opos6ul/O_LedBarTest.cpp
        src/bot-opos6ul/O_TiretteTest.cpp
        src/bot-opos6ul/O_ButtonBarTest.cpp
        src/bot-opos6ul/O_LcdBoardTest.cpp
        src/bot-opos6ul/O_ActionManagerTimerTest.cpp
        src/bot-opos6ul/O_IAbyPathTest.cpp
        src/bot-opos6ul/O_AsservLineRotateTest.cpp
        src/bot-opos6ul/O_AsservWaypointTest.cpp
        src/bot-opos6ul/O_NavigatorMovementTest.cpp
        src/bot-opos6ul/O_NavigatorBackTest.cpp
        src/bot-opos6ul/O_AsservXYRotateTest.cpp
        src/bot-opos6ul/O_AsservTest.cpp
        src/bot-opos6ul/O_AsservCalibrationTest.cpp
        src/bot-opos6ul/O_Asserv_SquareTest.cpp
        src/bot-opos6ul/O_Asserv_CalageTest.cpp
        src/bot-opos6ul/O_GroveColorTest.cpp
        src/bot-opos6ul/O_ServoStepTest.cpp
        src/bot-opos6ul/O_ServoObjectsTest.cpp
        src/bot-opos6ul/O_SensorsTest.cpp
    )

    target_include_directories(bot-opos6ul PRIVATE
        src/bot-opos6ul
    )

    target_link_libraries(bot-opos6ul PRIVATE pmx-common ${DRIVER_LIB} pmr-pathfinding)

    # Copie des fichiers nécessaires à côté de l'exécutable après le build
    add_custom_command(TARGET bot-opos6ul POST_BUILD
        # Config hardware (active/desactive les drivers)
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/src/bot-opos6ul/hardware.conf
            $<TARGET_FILE_DIR:bot-opos6ul>/hardware.conf
        # Config asservissement (chargé en relatif par OPOS6UL_AsservExtended)
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/src/bot-opos6ul/config_OPOS6UL_Robot.txt
            $<TARGET_FILE_DIR:bot-opos6ul>/config_OPOS6UL_Robot.txt
        # Ressources SVG pour le log (index.html + table directement dans bin/)
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/src/common/log/resources-svg/index.html
            $<TARGET_FILE_DIR:bot-opos6ul>/index.html
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/src/common/log/resources-svg/table.png
            $<TARGET_FILE_DIR:bot-opos6ul>/table.png
        COMMENT "Copie hardware.conf + config asserv + index.html + table.png dans bin/"
    )
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
    test/common/ActionManagerTimerTest.cpp
    test/common/RetryPolicyTest.cpp
    test/common/TableGeometryTest.cpp
    test/common/ObstacleZoneTest.cpp
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
        test/manual/ActionManagerTimerManualTest.cpp
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
        test/bench/PosixTimerBench.cpp
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
