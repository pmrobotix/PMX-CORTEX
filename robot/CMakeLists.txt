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
)

target_include_directories(pmx-common PUBLIC
    src/common
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
    )

    target_link_libraries(manual-test PRIVATE pmx-suite ${DRIVER_LIB})

    # =============================================
    # Cible test : bench (metriques de performance)
    # =============================================
    add_executable(bench
        test/bench/Main.cpp
        test/bench/LoggerInitialize.cpp
    )

    target_link_libraries(bench PRIVATE pmx-suite)
endif()
