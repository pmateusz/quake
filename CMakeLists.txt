#
# Copyright 2018 Mateusz Polnik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(quake LANGUAGES C CXX VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_VERBOSE_MAKEFILE FALSE)

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -DDEBUG -DUSE_GUROBI -DUSE_GLPK -DUSE_CBC")
    set(CMAKE_CXX_FLAGS_RELEASE "-O4 -DNDEBUG -march=native -DUSE_GUROBI -DUSE_GLPK -DUSE_CBC")
endif ()

set(default_build_type "DEBUG")
if (EXISTS "${CMAKE_SOURCE_DIR}/.git")
    set(default_build_type "DEBUG")
endif ()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "DEBUG" "RELEASE")
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/")

get_filename_component(GUROBI_ROOT_DIR /opt/gurobi811 REALPATH)
get_filename_component(ORTOOLS_ROOT_DIR ~/dev/or-tools-7.1 REALPATH)
get_filename_component(Protobuf_LIBRARY ${ORTOOLS_ROOT_DIR}/dependencies/install/lib REALPATH)
get_filename_component(Protobuf_PROTOC_EXECUTABLE ${ORTOOLS_ROOT_DIR}/dependencies/install/bin/protoc REALPATH)
get_filename_component(Protobuf_SRC_ROOT_FOLDER ${ORTOOLS_ROOT_DIR}/dependencies/sources/protobuf-3.6.1/src REALPATH)
get_filename_component(Protobuf_INCLUDE_DIR ${ORTOOLS_ROOT_DIR}/dependencies/install/include REALPATH)
get_filename_component(GFLAGS_ROOT_DIR ${ORTOOLS_ROOT_DIR}/dependencies/install REALPATH)
get_filename_component(GLOG_ROOT_DIR ${ORTOOLS_ROOT_DIR}/dependencies/install REALPATH)
get_filename_component(ABSL_ROOT_DIR ${ORTOOLS_ROOT_DIR}/dependencies/install REALPATH)

set_property(GLOBAL PROPERTY EXTERNAL_PROJECT_DEPENDENCIES "")

include(ExternalProject)
include(${CMAKE_SOURCE_DIR}/cmake/external/gflags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/external/glog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/external/gtest.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/external/json.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/external/pykep.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/external/date.cmake)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem regex graph)
find_package(Gurobi REQUIRED)
find_package(ortools REQUIRED)

set(HEADER_DEP ${Boost_INCLUDE_DIR}
        ${DATE_INCLUDE_DIR}
        ${CURL_INCLUDE_DIR}
        ${ORTOOLS_INCLUDE_DIR}
        ${ABSL_INCLUDE_DIR}
        ${GUROBI_INCLUDE_DIR}
        ${PYKEP_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIR}
        ${GFLAGS_INCLUDE_DIR}
        ${JSON_INCLUDE_DIR})

set(LIBRARY_DEP ${PYKEP_LIBRARY}
        ${DATE_LIBRARY}
        ${CURL_LIBRARY}
        ${ORTOOLS_LIBRARY}
        ${ABSL_LIBRARY}
        ${GUROBI_STATIC_LIBRARY}
        ${GUROBI_LIBRARY}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARY}
        ${CMAKE_THREAD_LIBS_INIT})

set(TEST_LIBRARY_DEP ${GTEST_LIBRARIES})

get_filename_component(HEADERS "${CMAKE_SOURCE_DIR}/src/main" REALPATH)

file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/src/main/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/main/index/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/main/robust/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/main/legacy/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/main/util/*.cpp")

add_library(quake STATIC ${SOURCES} src/main/util/datetime.h)
target_include_directories(quake PUBLIC ${HEADERS} ${HEADER_DEP})
add_executable(quake-cp ${CMAKE_SOURCE_DIR}/src/main/executables/quake_cp.cpp)
add_executable(quake-saa ${CMAKE_SOURCE_DIR}/src/main/executables/quake_saa.cpp)
add_executable(quake-mip ${CMAKE_SOURCE_DIR}/src/main/executables/quake_mip.cpp)
add_executable(quake-weights ${CMAKE_SOURCE_DIR}/src/main/executables/quake_edge_weights.cpp)
add_executable(quake-cross-moment ${CMAKE_SOURCE_DIR}/src/main/executables/quake_cross_moment.cpp)
add_executable(quake-cvar ${CMAKE_SOURCE_DIR}/src/main/executables/quake_cvar.cpp)
add_executable(quake-generate ${CMAKE_SOURCE_DIR}/src/main/executables/quake_generate.cpp)
add_executable(quake-robust ${CMAKE_SOURCE_DIR}/src/main/executables/quake_robust.cpp)
add_executable(quake-det ${CMAKE_SOURCE_DIR}/src/main/executables/quake_deterministic.cpp)
target_link_libraries(quake-cp quake ${LIBRARY_DEP})
target_link_libraries(quake-saa quake ${LIBRARY_DEP})
target_link_libraries(quake-mip quake ${LIBRARY_DEP})
target_link_libraries(quake-weights quake ${LIBRARY_DEP})
target_link_libraries(quake-cross-moment quake ${LIBRARY_DEP})
target_link_libraries(quake-cvar quake ${LIBRARY_DEP})
target_link_libraries(quake-generate quake ${LIBRARY_DEP})
target_link_libraries(quake-robust quake ${LIBRARY_DEP})
target_link_libraries(quake-det quake ${LIBRARY_DEP})

get_property(external_project_dependencies GLOBAL PROPERTY EXTERNAL_PROJECT_DEPENDENCIES)
#separate_arguments(external_project_dependencies)
foreach (project_dependency ${external_project_dependencies})
    add_dependencies(quake ${project_dependency})
endforeach ()

get_filename_component(HEADERS_TEST "${CMAKE_SOURCE_DIR}/src/test" REALPATH)

file(GLOB_RECURSE SOURCES_TEST "${CMAKE_SOURCE_DIR}/src/test/*.cpp")
add_library(quake-test STATIC ${SOURCES_TEST})
target_include_directories(quake-test PUBLIC ${HEADERS_TEST} ${HEADERS} ${HEADER_DEP} ${GTEST_INCLUDE_DIRS})
add_dependencies(quake-test quake)

if (GTEST_EXTERNAL)
    add_dependencies(quake-test gtest-external)
endif ()

macro(m_add_executable GOAL SOURCE)
    add_executable(${GOAL} ${SOURCE})
    target_include_directories(${GOAL} PUBLIC ${HEADERS_TEST} ${HEADERS} ${HEADER_DEP} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(${GOAL} quake-test ${TEST_LIBRARY_DEP} quake ${LIBRARY_DEP})
    set_property(TARGET ${GOAL} PROPERTY CXX_STANDARD 14)
    add_dependencies(${GOAL} quake-test)
endmacro()

file(GLOB_RECURSE _test_files src/test/*_test.cpp)
foreach (_test_file ${_test_files})
    get_filename_component(_test_file_name ${_test_file} NAME_WE)
    m_add_executable(${_test_file_name} ${_test_file})
endforeach ()

mark_as_advanced(_test_file_name _test_file _test_files)
