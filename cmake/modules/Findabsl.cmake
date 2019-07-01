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

if (NOT ABSL_ROOT_DIR)
    set(ABSL_ROOT_DIR "" CACHE PATH "Folder contains Gflags")
endif ()

if (ABSL_ROOT_DIR)
    set(_ABSL_INCLUDE_LOCATIONS "${ABSL_ROOT_DIR}/include")
    set(_ABSL_LIB_LOCATIONS "${ABSL_ROOT_DIR}/lib")
else ()
    set(_ABSL_INCLUDE_LOCATIONS "")
    set(_ABSL_LIB_LOCATIONS "")
endif ()

find_path(ABSL_INCLUDE_DIR absl/strings/ascii.h
        HINTS "${_ABSL_INCLUDE_LOCATIONS}")

find_library(absl_hash absl_hash
        HINTS "${_ABSL_LIB_LOCATIONS}")

find_library(absl_raw_hash_set absl_raw_hash_set
        HINTS "${_ABSL_LIB_LOCATIONS}")

find_library(absl_base absl_base
        HINTS "${_ABSL_LIB_LOCATIONS}")

set(ABSL_LIBRARY "${absl_hash};${absl_raw_hash_set};${absl_base}")
set(ABSL_LIBRARIES ${ABSL_LIBRARY})

find_package_handle_standard_args(ABSL DEFAULT_MSG ABSL_INCLUDE_DIR ABSL_LIBRARY ABSL_LIBRARIES)