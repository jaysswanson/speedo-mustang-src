# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(speedo_mustang C CXX ASM)

# ---------------------------------------------------------------------------
# Host-side unit tests (no Pico SDK required)
# Usage: cmake -DSPEEDO_UNIT_TEST=ON -DCMAKE_BUILD_TYPE=Debug ..
# ---------------------------------------------------------------------------
option(SPEEDO_UNIT_TEST "Build host-side unit tests instead of firmware" OFF)

if(SPEEDO_UNIT_TEST)
    # Plain host executable — no Pico SDK, no cross-compiler needed
    add_library(speedo_logic STATIC speedo_logic.cpp)
    target_include_directories(speedo_logic PUBLIC ${CMAKE_CURRENT_LIST_DIR})

    find_package(GTest REQUIRED)
    add_executable(speedo_tests test/test_speedo_logic.cpp)
    target_link_libraries(speedo_tests speedo_logic GTest::gtest GTest::gtest_main m)
    enable_testing()
    add_test(NAME speedo_unit_tests COMMAND speedo_tests)
    return()   # Skip everything below (Pico SDK not initialised in this mode)
endif()

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(speedo_mustang speedo_mustang.cpp speedo_logic.cpp)

pico_set_program_name(speedo_mustang "speedo_mustang")
pico_set_program_version(speedo_mustang "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(speedo_mustang 0)
pico_enable_stdio_usb(speedo_mustang 1)

# Add the standard library to the build
target_link_libraries(speedo_mustang
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(speedo_mustang PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(speedo_mustang 
        hardware_timer
        hardware_watchdog
        hardware_clocks
        )

pico_add_extra_outputs(speedo_mustang)

