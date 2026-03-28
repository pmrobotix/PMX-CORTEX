cmake_minimum_required(VERSION 3.21)
project(PMX-CORTEX LANGUAGES CXX)

# --- Standard C++ ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Options communes (comme dans Eclipse) ---
add_compile_options(-Wall -pthread)

# --- Plateforme : ARM ou SIMU ---
if(NOT TARGET_PLATFORM)
    set(TARGET_PLATFORM "SIMU")
endif()

message(STATUS "=== Plateforme cible : ${TARGET_PLATFORM} ===")

if(TARGET_PLATFORM STREQUAL "SIMU")
    # Mode simulation : compilation native Linux
    add_compile_definitions(SIMU)
    message(STATUS "Mode SIMU - compilation native")

elseif(TARGET_PLATFORM STREQUAL "ARM")
    # Mode ARM : cross-compilation OPOS6UL
    # Include path as_devices (comme dans Eclipse)
    set(AS_DEVICES_INCLUDE
        "/install/opos6ul-git/buildroot/output/host/arm-buildroot-linux-gnueabihf/sysroot/usr/include/as_devices")
    include_directories(${AS_DEVICES_INCLUDE})
    message(STATUS "Mode ARM - cross-compilation OPOS6UL")
endif()

# --- Define commune (comme dans Eclipse) ---
add_compile_definitions(CONFIG_MOTORCTRL_BOTMOTORS=1)

# =============================================
# Cible : test simple (validation de la config)
# =============================================
add_executable(pmx-test test/main_test.cpp)
target_link_libraries(pmx-test PRIVATE pthread)

# =============================================
# Tests manuels (hardware, a executer sur cible)
# =============================================
add_executable(test-screen test/manual/test_screen.cpp)
target_link_libraries(test-screen PRIVATE pthread)
