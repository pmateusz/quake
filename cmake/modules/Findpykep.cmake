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


if (NOT PYKEP_ROOT_DIR)
    set(PYKEP_ROOT_DIR "" CACHE PATH "Folder contains pykep library")
endif ()

if (PYKEP_ROOT_DIR)
    set(_PYKEP_INCLUDE_LOCATIONS "${PYKEP_ROOT_DIR}/include")
    set(_PYKEP_LIB_LOCATIONS "${PYKEP_ROOT_DIR}/lib")
else ()
    set(_PYKEP_INCLUDE_LOCATIONS "")
    set(_PYKEP_LIB_LOCATIONS "")
endif ()

find_path(PYKEP_INCLUDE_DIR pykep/pykep.h
        HINTS "${_PYKEP_INCLUDE_LOCATIONS}"
        PATHS "/usr/local/include")

find_library(PYKEPLIBRARY pykep
        HINTS  "${_PYKEP_LIB_LOCATIONS}"
        PATHS "/usr/local/lib/pykep"
        PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PYKEP DEFAULT_MSG PYKEP_INCLUDE_DIR PYKEP_LIBRARY)