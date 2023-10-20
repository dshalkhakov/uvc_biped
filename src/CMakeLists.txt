# CMakeList.txt : CMake project for biped_controller, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("biped_controller")

include_directories("${CMAKE_SOURCE_DIR}/sys_null/include")

# Add source to this project's executable.
add_library(biped_lib "sys_null/stubs.c" "main.c" "main.h")

add_executable (biped_controller ${BIPED_CONTROLLER_SRCS} "program.c" "main.h")
target_link_libraries(biped_controller biped_lib)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET biped_controller PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    add_subdirectory(tests)
endif ()

enable_testing()
