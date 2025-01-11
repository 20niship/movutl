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

include_directories(${MOVUTL_INCLUDES} ${MOVUTL_ROOT_DIR}/ext/doctest)
add_subdirectory(tests)
add_subdirectory(examples)

add_executable(movutl_main ${CMAKE_CURRENT_SOURCE_DIR}/lancher/main.cpp)
target_link_libraries(movutl_main mucore)

