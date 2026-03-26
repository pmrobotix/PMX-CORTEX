cmake_minimum_required(VERSION 3.21)
project(PMX-CORTEX LANGUAGES CXX)

# --- Standard C++ ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    src/common/utils/WString.cpp
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
    # utils (Arduino Print)
    src/common/utils/Print.cpp
)

target_include_directories(pmx-common PUBLIC
    src/common
    src/common/asserv.esial
)

target_link_libraries(pmx-common PUBLIC pthread rt simple-svg)

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
        src/driver-arm/AsservDriver.cpp
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
    )

    target_include_directories(pmx-driver-arm PUBLIC
        src/driver-arm
    )

    target_link_libraries(pmx-driver-arm PUBLIC pmx-common as_devices as_devices_cpp)

    set(DRIVER_LIB pmx-driver-arm)
else()
    # --- pmx-driver-simu (compilation native PC) ---
    add_library(pmx-driver-simu STATIC
        src/driver-simu/LedDriver.cpp
        src/driver-simu/ButtonDriver.cpp
        src/driver-simu/SwitchDriver.cpp
        src/driver-simu/SoundDriver.cpp
        src/driver-simu/LcdShieldDriver.cpp
        src/driver-simu/RobotPositionShared.cpp
        src/driver-simu/AsservDriver.cpp
        src/driver-simu/SensorsDriver.cpp
        src/driver-simu/ServoDriver.cpp
        src/driver-simu/ServoUsingMotorDriver.cpp
        src/driver-simu/ActionDriver.cpp
        src/driver-simu/ColorDriver.cpp
    )

    target_include_directories(pmx-driver-simu PUBLIC
        src/driver-simu
    )

    target_link_libraries(pmx-driver-simu PUBLIC pmx-common)

    set(DRIVER_LIB pmx-driver-simu)
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
    )

    target_link_libraries(driver-test PRIVATE pmx-suite ${DRIVER_LIB})

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
