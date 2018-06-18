cmake_minimum_required(VERSION 2.8.11)

# Cross Compile
if ("$ENV{CROSS_COMPILE_FLAG}" STREQUAL "XU4")
  message("Cross Compile For XU4")
  SET(CMAKE_SYSTEM_NAME Linux)
  SET(CMAKE_SYSTEM_PROCESSOR armv7l)
  SET(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
  SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
  SET(CMAKE_BUILD_TYPE Release)
endif()

# If on OSX, force cmake to use gcc-6/5 instead of the built-in gcc (AppleClang)
# The compiler overriden has to happen before calling "project"
if (APPLE)
  if (EXISTS /usr/local/bin/g++-6)
    message("Override CXX compiler to /usr/local/bin/g++-6!")
    set(CMAKE_C_COMPILER   /usr/local/bin/gcc-6)
    set(CMAKE_CXX_COMPILER /usr/local/bin/g++-6)
  elseif (EXISTS /usr/local/bin/g++-5)
    message("Override CXX compiler to /usr/local/bin/g++-5!")
    set(CMAKE_C_COMPILER   /usr/local/bin/gcc-5)
    set(CMAKE_CXX_COMPILER /usr/local/bin/g++-5)
  elseif (EXISTS /usr/local/bin/g++-7)
    message("Override CXX compiler to /usr/local/bin/g++-7!")
    set(CMAKE_C_COMPILER   /usr/local/bin/gcc-7)
    set(CMAKE_CXX_COMPILER /usr/local/bin/g++-7)
  else ()
    message(FATAL_ERROR "Cannot find GNU gcc on Mac!")
  endif()
  set(CMAKE_MACOSX_RPATH 0)
endif()

project(Solver)

# Configure cxx flags
if(CYGWIN)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -msse2 -Wno-unused-result")
else()
# NEON or SSE
if((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm"))
  set(CMAKE_CXX_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -mfloat-abi=hard -mfpu=neon -Ofast -std=c++11 -Wno-unused-result")
  message("Enabling neon for armv7 " ${CMAKE_CXX_FLAGS})
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
  set(CMAKE_CXX_FLAGS "-D__ARM_NEON__ -DENABLE_NEON -Ofast -std=c++11 -Wno-unused-result")
  message("Detected armv8 " ${CMAKE_CXX_FLAGS})
else()
 # SSE4.1 seems to be not supported by GCC 4.8.4
 # TODO(mingyu): See if we still need sse4.1
 message(STATUS "Enable SSE2 and SSE4.1")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse2 -msse4.1 -Wno-unused-result")
 # Export compile_commands.json
 set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
endif()
endif()

# Suppress warnings for deprecated declarations for GCC 5 and above
if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.0)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif()

# Debug / Release / RelWithDebInfo
set(CMAKE_BUILD_TYPE Release)

# Thirdparty dependencies
# If CFG_VIEW is turned on, need to install glut (freeglut3-dev) and glew (libglew-dev)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(CVD)
find_package(Boost COMPONENTS thread system filesystem REQUIRED)
find_package(Eigen REQUIRED)
find_package(Glog REQUIRED)
if (CVD_FOUND)
 add_definitions(-DIBA_WITH_CVD)
 find_package(OpenGL REQUIRED)
 find_package(GLUT REQUIRED)
 find_package(GLEW REQUIRED)
 if (UNIX)
  find_package(X11 REQUIRED)
 endif()
endif()

# Explicitly specify all header and source files
# =========================
# BundleAdjustment
# =========================
set(HEADERS_BUNDLEADJUSTMENT
 BundleAdjustment/GlobalBundleAdjustor.h
 BundleAdjustment/LocalBundleAdjustor.h
)
set(SOURCES_BUNDLEADJUSTMENT
 BundleAdjustment/GlobalBundleAdjustor.cpp
 BundleAdjustment/GlobalBundleAdjustorDebug.cpp
 BundleAdjustment/GlobalBundleAdjustorDL.cpp
 BundleAdjustment/GlobalBundleAdjustorGN.cpp
 BundleAdjustment/GlobalBundleAdjustorIO.cpp
 BundleAdjustment/LocalBundleAdjustor.cpp
 BundleAdjustment/LocalBundleAdjustorDebug.cpp
 BundleAdjustment/LocalBundleAdjustorDL.cpp
 BundleAdjustment/LocalBundleAdjustorGN.cpp
 BundleAdjustment/LocalBundleAdjustorIO.cpp
)

# =========================
# Geometry
# =========================
set(HEADERS_GEOMETRY
 Geometry/BoundingBox.h
 Geometry/Camera.h
 Geometry/CameraPrior.h
 Geometry/CameraTrajectory.h
 Geometry/Depth.h
 Geometry/IMU.h
 Geometry/Intrinsic.h
 Geometry/M-Estimator.h
 Geometry/Point.h
 Geometry/Rigid.h
 Geometry/Rotation.h
 Geometry/Similarity.h
)
set(SOURCES_GEOMETRY
 Geometry/Camera.cpp
 Geometry/CameraPrior.cpp
 Geometry/Depth.cpp
 Geometry/IMU.cpp
 Geometry/Intrinsic.cpp
)

# =========================
# LinearAlgebra
# =========================
set(LINEAR_ALGEBRA_INCLUDE_DIR
  "${PROJECT_SOURCE_DIR}/LinearAlgebra"
)
set(HEADERS_LINEARALGEBRA
 LinearAlgebra/LinearSystem.h
 LinearAlgebra/Matrix2x2.h
 LinearAlgebra/Matrix2x3.h
 LinearAlgebra/Matrix2x4.h
 LinearAlgebra/Matrix2x6.h
 LinearAlgebra/Matrix2x7.h
 LinearAlgebra/Matrix2x8.h
 LinearAlgebra/Matrix3x3.h
 LinearAlgebra/Matrix3x4.h
 LinearAlgebra/Matrix3x6.h
 LinearAlgebra/Matrix3x7.h
 LinearAlgebra/Matrix3x8.h
 LinearAlgebra/Matrix4x4.h
 LinearAlgebra/Matrix6x6.h
 LinearAlgebra/Matrix6x7.h
 LinearAlgebra/Matrix8x8.h
 LinearAlgebra/Matrix9x9.h
 LinearAlgebra/MatrixMxN.h
 LinearAlgebra/MatrixNxN.h
 LinearAlgebra/Vector12.h
 LinearAlgebra/Vector2.h
 LinearAlgebra/Vector3.h
 LinearAlgebra/Vector4.h
 LinearAlgebra/Vector6.h
 LinearAlgebra/Vector7.h
 LinearAlgebra/Vector8.h
 LinearAlgebra/Vector9.h
 LinearAlgebra/VectorN.h
)

# =========================
# Map
# =========================
set(HEADERS_MAP
 Map/Feature.h
 Map/Frame.h
 Map/GlobalMap.h
 Map/LocalMap.h
)
set(SOURCES_MAP
 Map/Feature.cpp
 Map/GlobalMap.cpp
 Map/LocalMap.cpp
)

# =========================
# Utility
# =========================
set(UTILITY_INCLUDE_DIR
  "${PROJECT_SOURCE_DIR}/Utility"
  )
set(HEADERS_UTILITY
 Utility/AlignedMatrix.h
 Utility/AlignedVector.h
 Utility/Candidate.h
 Utility/Configurator.h
 Utility/FillMap.h
 Utility/MultiThread.h
 Utility/SIMD.h
 Utility/simd_sse_neon.h
 Utility/Timer.h
 Utility/Utility.h
)
set(SOURCES_UTILITY
 Utility/Utility.cpp
 Utility/MultiThread.cpp
)

# =========================
# IBA library
# =========================
# TODO(mingyu): Move the include dir to code to be more clear,
# e.g., <IBA.h> becomes <IBA/IBA.h>
# Since IBA_INCLUDE_DIRS is relative, do NOT expose it outside of Solver
# TODO(mingyu): Need to add a cached IBA_INCLUDE_DIRS that only exposes the necessary headers
set(IBA_INCLUDE_DIRS
 IBA
 BundleAdjustment
 Geometry
 LinearAlgebra
 Map
 Utility
)
set(HEADERS_IBA
 IBA/IBA.h
 IBA/IBA_config.h
 IBA/IBA_datatype.h
 IBA/IBA_internal.h
 IBA/Parameter.h
 IBA/stdafx.h
 ${HEADERS_BUNDLEADJUSTMENT}
 ${HEADERS_GEOMETRY}
 ${HEADERS_LINEARALGEBRA}
 ${HEADERS_MAP}
 ${HEADERS_UTILITY}
)
set(SOURCES_IBA
 IBA/IBA.cpp
 IBA/Parameter.cpp
 IBA/stdafx.cpp
 ${SOURCES_BUNDLEADJUSTMENT}
 ${SOURCES_GEOMETRY}
 ${SOURCES_LINEARALGEBRA}
 ${SOURCES_MAP}
 ${SOURCES_UTILITY}
)
add_library(IBA SHARED
 ${SOURCES_IBA}
)
target_include_directories(IBA PUBLIC
 ${IBA_INCLUDE_DIRS}
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${EIGEN_INCLUDE_DIR}
 ${Boost_INCLUDE_DIR}
 ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(IBA
 ${Boost_LIBRARIES}
 ${GLOG_LIBRARIES}
 ${X11_LIBRARIES}
 pthread
)

# =========================
# Visualization
# =========================
if (CVD_FOUND)
set(VISUALIZATION_INCLUDE_DIR
 Visualization
)
set(HEADERS_VISUALIZATION
 Visualization/Arcball.h
 Visualization/EventHandler.h
 Visualization/Frustrum.h
 Visualization/Keyboard.h
 Visualization/TextureGL.h
 Visualization/TextureGL.hpp
 Visualization/UtilityWM.h
 Visualization/Viewer.h
 Visualization/ViewerIBA.h
)
set(SOURCES_VISUALIZATION
 Visualization/Viewer.cpp
 Visualization/ViewerIBA.cpp
 Visualization/ViewerIBADraw.cpp
 Visualization/ViewerIBAIO.cpp
)
add_library(IBAVis SHARED
 ${SOURCES_VISUALIZATION}
)
target_include_directories(IBAVis PUBLIC
 ${VISUALIZATION_INCLUDE_DIR}
 ${IBA_INCLUDE_DIRS}
 ${CVD_INCLUDE_DIRS}
 ${OPENGL_INCLUDE_DIR}
 ${GLUT_INCLUDE_DIRS}
 ${GLEW_INCLUDE_DIRS}
 ${CMAKE_CURRENT_SOURCE_DIR}
 ${EIGEN_INCLUDE_DIR}
)

target_link_libraries(IBAVis
 IBA
 ${Boost_LIBRARIES}
 ${CVD_LIBRARIES}
 ${OPENGL_LIBRARIES}
 ${GLUT_LIBRARY}
 ${GLEW_LIBRARIES}
 ${X11_LIBRARIES}
 Xrandr
 pthread
)
endif(CVD_FOUND)
