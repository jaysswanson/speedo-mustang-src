# CMakeLists.txt for the HIL test Pico firmware
# Flash this to the *second* Pico (the test driver), not the DUT.
#
# Build from the repo root:
#   cmake -B build-hil -G Ninja -DHIL_TEST=ON
#   cmake --build build-hil
#   # Output: build-hil/hil_test/hil_test.uf2

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Locate Pico SDK the same way the main project does
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Board type")
include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)

project(hil_test C CXX ASM)
pico_sdk_init()

add_executable(hil_test hil_test.cpp)

pico_set_program_name(hil_test "hil_test")
pico_set_program_version(hil_test "0.1")

pico_enable_stdio_uart(hil_test 0)
pico_enable_stdio_usb(hil_test 1)

target_include_directories(hil_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(hil_test
    pico_stdlib
    hardware_timer
    hardware_gpio
)

pico_add_extra_outputs(hil_test)
