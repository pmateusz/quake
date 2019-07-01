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

if (NOT __DATE_INCLUDED)
    set(__DATE_INCLUDED TRUE)

    find_package(date)
    if (DATE_FOUND)
        set(DATE_EXTERNAL FALSE)
    else ()
        # build directory
        set(date_PREFIX ${CMAKE_BINARY_DIR}/external/date-prefix)
        # install directory
        set(date_INSTALL ${CMAKE_BINARY_DIR}/external/date-install)

        ExternalProject_Add(date-external
                PREFIX ${date_PREFIX}
                URL_HASH SHA256=9278d7568afdf6fb880f277d5e344f92c0531459f15643bc945849c2f209dab2
                URL "https://github.com/HowardHinnant/date/archive/v2.4.1.zip"
                INSTALL_DIR ${date_INSTALL}
                PATCH_COMMAND ""
                UPDATE_COMMAND ""
                INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory ${date_INSTALL}/lib
                COMMAND ${CMAKE_COMMAND} -E make_directory ${date_INSTALL}/include/date
                COMMAND ${CMAKE_COMMAND} -E copy ${date_PREFIX}/src/date-external-build/libtz.a ${date_INSTALL}/lib
                COMMAND rsync -rv --include "*/" --include "*.h" --exclude "*" --prune-empty-dirs ${date_PREFIX}/src/date-external/include/ ${date_INSTALL}/include
                CMAKE_ARGS -DENABLE_DATE_TESTING=OFF -DCMAKE_CXX_STANDARD=14 -DBUILD_SHARED_LIBS=OFF -DUSE_TZ_DB_IN_DOT=OFF -DUSE_SYSTEM_TZ_DB=OFF)

        find_package(CURL REQUIRED)

        set(DATE_FOUND TRUE)
        set(DATE_LIBRARIES ${date_INSTALL}/lib/libtz.a)
        set(DATE_LIBRARY ${DATE_LIBRARIES})
        set(DATE_INCLUDE_DIRS ${date_INSTALL}/include)
        set(DATE_INCLUDE_DIR ${DATE_INCLUDE_DIRS})
        set(DATE_EXTERNAL TRUE)
        set_property(GLOBAL APPEND PROPERTY EXTERNAL_PROJECT_DEPENDENCIES date-external)
    endif ()

endif ()