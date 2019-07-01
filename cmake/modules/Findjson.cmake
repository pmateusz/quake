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

if (NOT JSON_ROOT_DIR)
    set(JSON_ROOT_DIR "" CACHE PATH "Folder contains JSON for modern C++")
endif ()

if (JSON_ROOT_DIR)
    set(_JSON_INCLUDE_LOCATIONS "${JSON_ROOT_DIR}/include")
else ()
    set(_JSON_INCLUDE_LOCATIONS "")
endif ()

find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(JSON_PKGCONF QUIET json)
endif ()

# We are testing only a couple of files in the include directories
find_path(JSON_INCLUDE_DIR nlohmann/json.hpp
        HINTS "${_JSON_INCLUDE_LOCATIONS}"
        PATHS "${JSON_PKGCONF_INCLUDE_DIRS}" "/usr/local/include")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JSON DEFAULT_MSG JSON_INCLUDE_DIR)

if (JSON_FOUND)
    set(JSON_INCLUDE_DIR ${JSON_INCLUDE_DIR})
endif ()

mark_as_advanced(JSON_LIBRARY_DEBUG JSON_LIBRARY_RELEASE _JSON_INCLUDE_LOCATIONS)