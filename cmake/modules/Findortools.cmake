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

if (NOT ORTOOLS_ROOT_DIR)
    set(ORTOOLS_ROOT_DIR "" CACHE PATH "Folder contains OR-tools library")
endif ()

if (ORTOOLS_ROOT_DIR)
    set(_ORTOOLS_INCLUDE_LOCATIONS "${ORTOOLS_ROOT_DIR}")
    set(_ORTOOLS_LIB_LOCATIONS "${ORTOOLS_ROOT_DIR}/lib")
    set(_ORTOOLS_INCLUDE_GEN_LOCATIONS "${ORTOOLS_ROOT_DIR}/ortools/gen")
else ()
    set(_ORTOOLS_INCLUDE_LOCATIONS "")
    set(_ORTOOLS_LIB_LOCATIONS "")
    set(_ORTOOLS_INCLUDE_GEN_LOCATIONS "")
endif ()

find_path(ORTOOLS_INCLUDE_DIR
        NAMES ortools/linear_solver/linear_solver.h
        HINTS ${_ORTOOLS_INCLUDE_LOCATIONS})

find_path(ORTOOLS_INCLUDE_GEN_DIR
        NAMES ortools/linear_solver/linear_solver.pb.h
        HINTS "${_ORTOOLS_INCLUDE_GEN_LOCATIONS}")

if (ORTOOLS_INCLUDE_GEN_DIR AND NOT (ORTOOLS_INCLUDE_GEN_DIR EQUAL ORTOOLS_INCLUDE_DIR))
    list(APPEND ORTOOLS_INCLUDE_DIR ${ORTOOLS_INCLUDE_GEN_DIR})
endif ()

find_library(ORTOOLS_LIBRARY
        NAMES ortools
        HINTS ${_ORTOOLS_LIB_LOCATIONS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORTOOLS DEFAULT_MSG ORTOOLS_INCLUDE_DIR ORTOOLS_LIBRARY)

if (ORTOOLS_FOUND)
    find_package(Protobuf REQUIRED)
    find_package(absl REQUIRED)

    list(APPEND ORTOOLS_INCLUDE_DIR ${Protobuf_INCLUDE_DIR})
    list(APPEND ORTOOLS_INCLUDE_DIR ${ABSL_INCLUDE_DIR})
endif ()

mark_as_advanced(_ORTOOLS_INCLUDE_LOCATIONS _ORTOOLS_INCLUDE_GEN_LOCATIONS _ORTOOLS_LIB_LOCATIONS)