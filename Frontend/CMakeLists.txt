cmake_minimum_required(VERSION 2.8.11)
project(euroc_tracking)

list(FIND OpenCV_LIBS "opencv_viz" find_index)
if (find_index GREATER -1)
    message(STATUS "Found opencv_viz")
    add_definitions(-DHAS_OPENCV_VIZ)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Yaml REQUIRED)

#include_directories (${GLOG_INCLUDE_DIR})

set(HEADERS
        cameras/CameraBase.hpp
        cameras/DistortionBase.hpp
        cameras/PinholeCamera.hpp
        cameras/RadialTangentialDistortion.hpp
        cameras/RadialTangentialDistortion8.hpp
        cameras/implementation/CameraBase.hpp
        cameras/implementation/PinholeCamera.hpp
        cameras/implementation/RadialTangentialDistortion8.hpp
        cameras/implementation/RadialTangentialDistortion.hpp
        basic_datatype.h
        feature_utils.h
        iba_helper.h
        ORBextractor.h
        param.h
        patch_score.h
        plotting_utils.h
        pose_viewer.h
        rotation.h
        shared_queue.h
        xppyramid.hpp
        image_utils.h
        timer.h
        xp_quaternion.h
        )

set(SOURCES
    feature_utils.cc
    feature_utils_direct_matcher.cc
    feature_utils_feature_track.cc
    feature_utils_warp.cc
    feature_utils_uniformity.cc
    feature_utils_align.cc
    iba_helper.cc
    ORBextractor.cc
    plotting_utils.cpp
    pose_viewer.cc
    timer.cc
    rotation.cc
    param.cc
    xppyramid.cpp
    image_utils.cc
    CameraBase.cpp
    xp_quaternion.cc
    )

add_library(OF SHARED
    ${SOURCES}
)
target_include_directories(OF PUBLIC
    ${GFLAGS_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras
)
target_link_libraries(OF
    ${GFLAGS_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAMLCPP_LIBRARIES}
    IBA
    glog
)
