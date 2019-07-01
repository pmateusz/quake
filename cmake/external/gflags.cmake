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


if (NOT __GFLAGS_INCLUDED) # guard against multiple includes
    set(__GFLAGS_INCLUDED TRUE)

    # use the system-wide gflags if present
    find_package(gflags)
    if (GFLAGS_FOUND)
        set(GFLAGS_EXTERNAL FALSE)
    else()
        # gflags will use pthreads if it's available in the system, so we must link with it
        find_package(Threads)

        # build directory
        set(gflags_PREFIX ${CMAKE_BINARY_DIR}/external/gflags-prefix)
        # install directory
        set(gflags_INSTALL ${CMAKE_BINARY_DIR}/external/gflags-install)

        # we build gflags statically
        if (UNIX)
            set(GFLAGS_EXTRA_COMPILER_FLAGS "-fPIC")
        endif()

        set(GFLAGS_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${GFLAGS_EXTRA_COMPILER_FLAGS})
        set(GFLAGS_C_FLAGS ${CMAKE_C_FLAGS} ${GFLAGS_EXTRA_COMPILER_FLAGS})

        ExternalProject_Add(gflags-external
                PREFIX ${gflags_PREFIX}
                GIT_REPOSITORY "https://github.com/gflags/gflags.git"
                GIT_TAG "v2.2.1"
                UPDATE_COMMAND ""
                INSTALL_DIR ${gflags_INSTALL}
                CMAKE_ARGS -DCMAKE_BUILD_TYPE=RELEASE
                -DCMAKE_INSTALL_PREFIX=${gflags_INSTALL}
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_STATIC_LIBS=ON
                -DBUILD_PACKAGING=OFF
                -DBUILD_TESTING=OFF
                -DBUILD_NC_TESTS=OFF
                -BUILD_CONFIG_TESTS=OFF
                -DINSTALL_HEADERS=ON
                -DCMAKE_C_FLAGS=${GFLAGS_C_FLAGS}
                -DCMAKE_CXX_FLAGS=${GFLAGS_CXX_FLAGS}
                LOG_DOWNLOAD 1
                LOG_CONFIGURE 1
                LOG_INSTALL 1)

        set(GFLAGS_FOUND TRUE)
        set(GFLAGS_ROOT_DIR ${gflags_INSTALL})
        set(GFLAGS_INCLUDE_DIRS ${gflags_INSTALL}/include)
        set(GFLAGS_INCLUDE_DIR ${GFLAGS_INCLUDE_DIRS})
        set(GFLAGS_LIBRARIES ${gflags_INSTALL}/lib/libgflags.a ${CMAKE_THREAD_LIBS_INIT})
        set(GFLAGS_LIBRARY ${GFLAGS_LIBRARIES})
        set(GFLAGS_LIBRARY_DIRS ${gflags_INSTALL}/lib)
        set(GFLAGS_EXTERNAL TRUE)
        set_property(GLOBAL APPEND PROPERTY EXTERNAL_PROJECT_DEPENDENCIES gflags-external)
    endif()

endif()