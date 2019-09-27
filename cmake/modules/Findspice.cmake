#
# Copyright 2019 Mateusz Polnik
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


if (NOT SPICE_ROOT_DIR)
    set(SPICE_ROOT_DIR "" CACHE PATH "Folder contains pykep library")
endif ()

if (SPICE_ROOT_DIR)
    set(_SPICE_INCLUDE_LOCATIONS "${SPICE_ROOT_DIR}/include")
    set(_SPICE_LIB_LOCATIONS "${SPICE_ROOT_DIR}/lib")
else ()
    set(_SPICE_INCLUDE_LOCATIONS "")
    set(_SPICE_LIB_LOCATIONS "")
endif ()

find_path(SPICE_INCLUDE_DIR f2c.h
        HINTS "${_SPICE_INCLUDE_LOCATIONS}")
#
#find_library(PYKEPLIBRARY pykep
#        HINTS  "${_PYKEP_LIB_LOCATIONS}"
#        PATHS "/usr/local/lib/pykep"
#        PATH_SUFFIXES lib)
#
#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(PYKEP DEFAULT_MSG PYKEP_INCLUDE_DIR PYKEP_LIBRARY)