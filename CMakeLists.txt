cmake_minimum_required(VERSION 3.15)
project(Sober_on_board_module LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--------------------------------------------------
# CAMERA HARDWARE ALWAYS ENABLED
#--------------------------------------------------
add_compile_definitions(SOBER_ENABLE_HW_CAMERAS=1)
message(STATUS "Hardware Cameras ENABLED (Spinnaker + eBUS)")

#--------------------------------------------------
# Spinnaker SDK (Pleora compatible FLIR SDK)
#--------------------------------------------------
set(SPINNAKER_ROOT "/opt/spinnaker" CACHE PATH "Path to Spinnaker SDK")

find_library(SPINNAKER_LIB Spinnaker
    PATHS "${SPINNAKER_ROOT}/lib" "${SPINNAKER_ROOT}/lib64"
    REQUIRED
)

add_library(Spinnaker::Spinnaker SHARED IMPORTED GLOBAL)
set_target_properties(Spinnaker::Spinnaker PROPERTIES
    IMPORTED_LOCATION "${SPINNAKER_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${SPINNAKER_ROOT}/include"
)

#--------------------------------------------------
# eBUS SDK (Pleora Telops / JAI)
#--------------------------------------------------
set(EBUS_ROOT "$ENV{PUREGEV_ROOT}" CACHE PATH "Path to eBUS SDK")

if (NOT EXISTS "${EBUS_ROOT}/include")
    message(FATAL_ERROR "Invalid EBUS_ROOT: '${EBUS_ROOT}' – run: source set_puregev_env")
endif()

find_path(EBUS_INCLUDE_DIR
    NAMES PvDevice.h PvStream.h PvBuffer.h
    HINTS "${EBUS_ROOT}/include"
    REQUIRED
)

find_library(EBUS_PVDEVICE NAMES PvDevice HINTS "${EBUS_ROOT}/lib" REQUIRED)
find_library(EBUS_PVSTREAM NAMES PvStream HINTS "${EBUS_ROOT}/lib" REQUIRED)
find_library(EBUS_PVBASE   NAMES PvBase   HINTS "${EBUS_ROOT}/lib" REQUIRED)
find_library(EBUS_PVBUFFER NAMES PvBuffer HINTS "${EBUS_ROOT}/lib" REQUIRED)
find_library(EBUS_PVGENICAM NAMES PvGenICam HINTS "${EBUS_ROOT}/lib" REQUIRED)
find_library(EBUS_PVSYSTEM  NAMES PvSystem  HINTS "${EBUS_ROOT}/lib" REQUIRED)

add_library(ebus INTERFACE)
target_include_directories(ebus SYSTEM INTERFACE "${EBUS_INCLUDE_DIR}")
target_link_libraries(ebus INTERFACE
    ${EBUS_PVDEVICE}
    ${EBUS_PVSTREAM}
    ${EBUS_PVBASE}
    ${EBUS_PVBUFFER}
    ${EBUS_PVGENICAM}
    ${EBUS_PVSYSTEM}
)

#--------------------------------------------------
# External libraries: spdlog, lz4, periphery
#--------------------------------------------------
# spdlog (header only)
add_subdirectory(3rdparty/spdlog)

# lz4 (required for Telops camera)
add_subdirectory(3rdparty/lz4/build/cmake EXCLUDE_FROM_ALL)
add_library(lz4::lz4 ALIAS lz4_static)

# REMOVE: c-periphery (no sensors needed)
# REMOVE: SPSCQueue (no threading queue needed)

add_subdirectory(3rdparty/RingBuffer)
add_subdirectory(3rdparty/SPSCQueue)

#--------------------------------------------------
# Include directory for project modules
#--------------------------------------------------
include_directories(${CMAKE_SOURCE_DIR}/include)

#--------------------------------------------------
# Submodules you KEEP
#--------------------------------------------------
add_subdirectory(src/camera)
add_subdirectory(src/logger)
add_subdirectory(src/manager)
add_subdirectory(src/main)     # builds final executable
add_subdirectory(src/communicator)

#--------------------------------------------------
# RPATH for running natively on Raspberry Pi
#--------------------------------------------------
set(CMAKE_BUILD_RPATH
    "${SPINNAKER_ROOT}/lib"
    "${EBUS_ROOT}/lib"
)
