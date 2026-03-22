# Toolchain CMake pour cross-compilation ARM OPOS6UL
# Equivalent des réglages "Cross GCC" dans Eclipse

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Chemin vers la toolchain Armadeus/Buildroot
set(TOOLCHAIN_PATH "/install/opos6ul-git/buildroot/output/host")
set(TOOLCHAIN_PREFIX "arm-none-linux-gnueabihf-")

# Compilateurs
set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}g++")
set(CMAKE_STRIP        "${TOOLCHAIN_PATH}/bin/${TOOLCHAIN_PREFIX}strip")

# Sysroot (headers et libs du système cible)
set(CMAKE_SYSROOT "${TOOLCHAIN_PATH}/arm-buildroot-linux-gnueabihf/sysroot")

# Ne pas chercher les programmes sur la cible, mais chercher les libs/headers
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
