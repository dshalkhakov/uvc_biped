﻿# CMakeList.txt : CMake project for sim, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("sim")

include_directories("${CMAKE_SOURCE_DIR}/../../ode-0.16.3/include" "../src/sys_null/include" "../src")
link_directories("${CMAKE_SOURCE_DIR}/../../ode-0.16.3/out/build/x64-debug")

add_library(biped_lib "../src/main.c" "../src/main.h" "../src/sys_sim/i2c.c" "../src/sys_win/timer.c" "../src/sys_sim/uart.c" "../src/sys_sim/ics.c" "../src/sys_sim/stubs.c")

# Add source to this project's executable.
add_executable (sim ${SIM_SRCS} "biped.cpp" "biped.h" "core.cpp" "core.h" "main_integration.cpp" "main_integration.h")
target_link_libraries(sim ode_doubled drawstuff biped_lib)
target_compile_definitions(sim PRIVATE USING_MAIN)

add_custom_command(
        TARGET sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/../../ode-0.16.3/out/build/x64-debug/ode_doubled.dll
                ${CMAKE_CURRENT_BINARY_DIR}/ode_doubled.dll)
add_custom_command(
        TARGET sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/../../ode-0.16.3/out/build/x64-debug/drawstuff.dll
                ${CMAKE_CURRENT_BINARY_DIR}/drawstuff.dll)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET sim PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
