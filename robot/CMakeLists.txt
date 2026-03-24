cmake_minimum_required(VERSION 3.21)
project(PMX-CORTEX LANGUAGES CXX)

# --- Standard C++ ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Options communes ---
add_compile_options(-Wall -pthread)

# --- Plateforme : ARM ou SIMU ---
if(NOT TARGET_PLATFORM)
    set(TARGET_PLATFORM "SIMU")
endif()

message(STATUS "=== Plateforme cible : ${TARGET_PLATFORM} ===")

if(TARGET_PLATFORM STREQUAL "SIMU")
    add_compile_definitions(SIMU)
    message(STATUS "Mode SIMU - compilation native")
elseif(TARGET_PLATFORM STREQUAL "ARM")
    set(AS_DEVICES_INCLUDE
        "/install/opos6ul-git/buildroot/output/host/arm-buildroot-linux-gnueabihf/sysroot/usr/include/as_devices")
    include_directories(${AS_DEVICES_INCLUDE})
    message(STATUS "Mode ARM - cross-compilation OPOS6UL")
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
    # log
    src/common/log/Level.cpp
    src/common/log/Logger.cpp
    src/common/log/LoggerFactory.cpp
    src/common/log/appender/MemoryAppender.cpp
    src/common/log/appender/ConsoleAppender.cpp
)

target_include_directories(pmx-common PUBLIC
    src/common
)

target_link_libraries(pmx-common PUBLIC pthread rt)

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
)

target_link_libraries(common-test PRIVATE pmx-suite)
