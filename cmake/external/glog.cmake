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


# glog depends on gflags
include("${CMAKE_SOURCE_DIR}/cmake/external/gflags.cmake")

if (NOT __GLOG_INCLUDED)
    set(__GLOG_INCLUDED TRUE)

    # try the system-wide glog first
    find_package(Glog)
    if (GLOG_FOUND)
        set(GLOG_EXTERNAL FALSE)
    else ()
        # fetch and build glog from github

        # build directory
        set(glog_PREFIX ${CMAKE_BINARY_DIR}/external/glog-prefix)
        # install directory
        set(glog_INSTALL ${CMAKE_BINARY_DIR}/external/glog-install)

        # we build glog statically
        if (UNIX)
            set(GLOG_EXTRA_COMPILER_FLAGS "-fPIC")
        endif ()

        set(GLOG_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS})
        set(GLOG_C_FLAGS ${CMAKE_C_FLAGS} ${GLOG_EXTRA_COMPILER_FLAGS})

        # depend on gflags if we're also building it
        if (GFLAGS_EXTERNAL)
            set(GLOG_DEPENDS gflags-external)
        endif ()

        ExternalProject_Add(glog-external
                DEPENDS ${GLOG_DEPENDS}
                PREFIX ${glog_PREFIX}
                GIT_REPOSITORY "https://github.com/google/glog.git"
                GIT_TAG "v0.3.5"
                UPDATE_COMMAND ""
                INSTALL_DIR ${glog_INSTALL}
                CMAKE_ARGS -DCMAKE_BUILD_TYPE=RELEASE
                -DCMAKE_INSTALL_PREFIX=${glog_INSTALL}
                -DBUILD_SHARED_LIBS=OFF
                -DBUILD_STATIC_LIBS=ON
                -DBUILD_TESTING=OFF
                -DBUILD_NC_TESTS=OFF
                -BUILD_CONFIG_TESTS=OFF
                -DINSTALL_HEADERS=ON
                -DCMAKE_C_FLAGS=${GFLAGS_C_FLAGS}
                -DCMAKE_CXX_FLAGS=${GFLAGS_CXX_FLAGS}
                -DWITH_GFLAGS=ON
                -DBUILD_NC_TESTS=OFF
                -DBUILD_STATIC_LIBS=ON
#                PATCH_COMMAND autoreconf -i ${glog_PREFIX}/src/glog-external
#                CONFIGURE_COMMAND env "CFLAGS=${GLOG_C_FLAGS}" "CXXFLAGS=${GLOG_CXX_FLAGS}" ${glog_PREFIX}/src/glog-external/configure --prefix=${glog_INSTALL} --enable-shared=no --enable-static=yes --with-gflags=${GFLAGS_ROOT_DIR}
                LOG_DOWNLOAD 1
                LOG_CONFIGURE 1
                LOG_INSTALL 1)

        set(GLOG_FOUND TRUE)
        set(GLOG_INCLUDE_DIRS ${glog_INSTALL}/include)
        set(GLOG_INCLUDE_DIR ${GLOG_INCLUDE_DIRS})
        set(GLOG_LIBRARIES ${GFLAGS_LIBRARIES} ${glog_INSTALL}/lib/libglog.a)
        set(GLOG_LIBRARY_DIRS ${glog_INSTALL}/lib)
        set(GLOG_EXTERNAL TRUE)

        if (GFLAGS_EXTERNAL)
            add_dependencies(glog-external gflags-external)
        endif ()
        set_property(GLOBAL APPEND PROPERTY EXTERNAL_PROJECT_DEPENDENCIES glog-external)
    endif ()

endif ()