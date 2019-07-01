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

if (NOT DATE_ROOT_DIR)
    set(DATE_ROOT_DIR "" CACHE PATH "Folder contains 'date' library")
endif ()

if (DATE_ROOT_DIR)
    set(_DATE_INCLUDE_LOCATIONS "${DATE_ROOT_DIR}/include")
    set(_DATE_LIB_LOCATIONS "${DATE_ROOT_DIR}/lib")
else ()
    set(_DATE_INCLUDE_LOCATIONS "")
    set(_DATE_LIB_LOCATIONS "")
endif ()

find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(DATE_PKGCONF QUIET date)
endif ()

# We are testing only a couple of files in the include directories
find_path(DATE_INCLUDE_DIR date/zt.hpp
        HINTS "${_DATE_INCLUDE_LOCATIONS}"
        PATHS "${DATE_PKGCONF_INCLUDE_DIRS}" "/usr/local/include")

find_library(TZ_LIBRARY tz
        HINTS ${_DATE_LIB_LOCATIONS}
        PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DATE DEFAULT_MSG DATE_INCLUDE_DIR TZ_LIBRARY)

if (DATE_FOUND)
    find_package(CURL REQUIRED)

    set(DATE_INCLUDE_DIR ${DATE_INCLUDE_DIR})
    set(DATE_LIBRARIES ${TZ_LIBRARY} ${CURL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    set(DATE_LIBRARY ${DATE_LIBRARIES})
    set(DATE_INCLUDE_DIR ${DATE_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})
    set(DATE_INCLUDE_DIRS ${DATE_INCLUDE_DIR})
endif ()

mark_as_advanced(DATE_LIBRARY_DEBUG DATE_LIBRARY_RELEASE _DATE_INCLUDE_LOCATIONS)