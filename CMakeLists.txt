# Copyright (c) 2018-20, Polaris3D. All rights reserved.

cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

project(vmrs)

SET(glui_DIR /home/nvidia/Downloads/glui/build/generated/)
list(APPEND CMAKE_PREFIX_PATH /home/nvidia/Downloads/glui)
# Check that build platform is TX2 or not.
execute_process( COMMAND uname -r COMMAND grep "jetson" OUTPUT_VARIABLE SYS_ARCH_RELEASE)
execute_process( COMMAND uname -r COMMAND grep "tegra" OUTPUT_VARIABLE SYS_ARCH_RELEASE_TEGRA)
option(BUILD_TX2 "Build this library on jetson TX2 (default: ON)" ON)
if(SYS_ARCH_RELEASE STREQUAL "" AND SYS_ARCH_RELEASE_TEGRA STREQUAL "")
  set(BUILD_TX2 OFF)
else()
  set(BUILD_TX2 ON)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")
option(BUILD_ANS_WARPPER "Build test codes for OS1 lidar (default: ON)" ON)
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../vgview)
  set(BUILD_ANS_WARPPER ON)
  else()
  set(BUILD_ANS_WARPPER OFF)
endif()
message("Build options,")
message(" - TX2:    " ${BUILD_TX2})
message(" - vgview: " ${BUILD_ANS_WARPPER})

# Find packages
find_package(dlib REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(glui REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

add_definitions(
  -std=c++14
  -O2
  -rdynamic
)
if (BUILD_TX2)
  ADD_DEFINITIONS(
    -D__tx2__
  )
endif()
if (BUILD_ANS_WARPPER)
  ADD_DEFINITIONS(
    -D__build_ans_wrapper__
  )
endif()

set(SOURCES
  src/RsD4xx.cc
)
if (BUILD_TX2)
  list(APPEND SOURCES
  )
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include_directories( 
  inc
  src
  ../vsys/inc
  ../vconfig/inc
  ../vlidaralgo/inc
  ../vgview/inc
  /usr/local/include/eigen3
  ${jsoncpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

set(TEST_COMMON_LIBS
  vmrs
  opencv_world
  realsense2
  glog
  gflags
)
set(TEST_3D_LIBS
  ${CMAKE_CURRENT_SOURCE_DIR}/../vgview/lib/libvgview.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../vsys/lib/libvsys.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../vconfig/lib/libvconfig.a
#  ${CMAKE_CURRENT_SOURCE_DIR}/../lastools/lib/liblastools.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../redisdb/lib/libredisdb.a
# acl redis-client
  ${CMAKE_CURRENT_SOURCE_DIR}/../acl/lib_acl_cpp/lib/lib_acl_cpp.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../acl/lib_protocol/lib/lib_protocol.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../acl/lib_acl/lib/lib_acl.a
  dlib
  pthread
  jsoncpp
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLUT_LIBRARY}
  ${GLEW_LIBRARIES}
  glui
  dl
)


# TEST ir and depth images
set(TEST_SHOW_OPENCV_SOURCES
  test/TEST_showOpenCv.cc
)
add_executable(TEST_showOpenCv ${TEST_SHOW_OPENCV_SOURCES})
target_link_libraries(TEST_showOpenCv
  ${TEST_COMMON_LIBS}
)

# TEST pointcloud visualization provided by librealsense
set(TEST_SHOW_POINTCLOUD_SOURCES
  test/TEST_showPointCloud.cc
)
add_executable(TEST_showPointCloud ${TEST_SHOW_POINTCLOUD_SOURCES})
target_link_libraries(TEST_showPointCloud
  ${TEST_COMMON_LIBS}
)

# TEST pointcloud cloud from depth image, using RsD4xx
set(TEST_RSD4XX_SOURCES
  test/TEST_rsd4xx.cc
)
add_executable(TEST_rsd4xx ${TEST_RSD4XX_SOURCES})
target_link_libraries(TEST_rsd4xx
  ${TEST_COMMON_LIBS}
)

if (BUILD_ANS_WRAPPER)
# TEST Points cloud from depth image 
set(TEST_SHOW_CLOUD_SOURCES
  test/TEST_showCloud.cc
)
add_executable(TEST_showCloud ${TEST_SHOW_CLOUD_SOURCES})
target_link_libraries(TEST_showCloud
  ${TEST_3D_LIBS}
  ${TEST_COMMON_LIBS}
)
endif()

# Do not use install because other codes reference this in locally, not globally.
install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION lib # ${CMAKE_INSTALL_LIBDIR}
  COMPONENT library
)

