cmake_minimum_required(VERSION 3.10...3.30)
project(movutl C CXX)

option(BUILD_DOC "Build documentation" OFF)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake)

if(BUILD_DOC)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/doxygen.cmake)
endif()

set(MOVUTL_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(${MOVUTL_ROOT_DIR}/ext/nanobind)
add_subdirectory(movutl)

message(STATUS "MOVUTL_INCLUDE_DIR: ${MOVUTL_INCLUDES}")
include_directories(${MOVUTL_INCLUDES})
add_subdirectory(tests)
add_subdirectory(examples)

add_executable(movutl_main ${CMAKE_CURRENT_SOURCE_DIR}/lancher/main.cpp)


