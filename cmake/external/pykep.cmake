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


if (NOT __PYKEP_INCLUDED)
    set(__PYKEP_ASTRO_INCLUDED TRUE)

    # try the system-wide glog first
    find_package(pykep)
    if (PYKEP_FOUND)
        set(PYKEP_EXTERNAL FALSE)
    else ()
        # fetch and build glog from github

        # build directory
        set(pykep_PREFIX ${CMAKE_BINARY_DIR}/external/pykep-prefix)
        # install directory
        set(pykep_INSTALL ${CMAKE_BINARY_DIR}/external/pykep-install)

        ExternalProject_Add(pykep-external
                PREFIX ${pykep_PREFIX}
                GIT_REPOSITORY "git@github.com:pmateusz/pykep.git"
                GIT_SHALLOW 1
                UPDATE_COMMAND ""
                INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory ${pykep_INSTALL}/lib
                COMMAND ${CMAKE_COMMAND} -E make_directory ${pykep_INSTALL}/include/pykep
                COMMAND ${CMAKE_COMMAND} -E copy ${pykep_PREFIX}/src/pykep-external-build/src/libkeplerian_toolbox_static.a ${pykep_INSTALL}/lib
                COMMAND rsync -rv --include "*/" --include "*.h" --exclude "*" --prune-empty-dirs ${pykep_PREFIX}/src/pykep-external/src/ ${pykep_INSTALL}/include/pykep
                CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=${pykep_INSTALL}
                LOG_DOWNLOAD 1
                LOG_CONFIGURE 1
                LOG_INSTALL 1)

        set(PYKEP_INCLUDE_DIR ${pykep_INSTALL}/include)
        set(PYKEP_LIBRARY ${pykep_INSTALL}/lib/libkeplerian_toolbox_static.a)
        set(PYKEP_FOUND TRUE)
        set(PYKEP_EXTERNAL TRUE)
        set_property(GLOBAL APPEND PROPERTY EXTERNAL_PROJECT_DEPENDENCIES pykep-external)
    endif ()

endif ()