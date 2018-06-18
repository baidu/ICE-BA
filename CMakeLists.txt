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

project(ICE_BA)

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
if (CVD_FOUND)
 add_definitions(-DIBA_WITH_CVD)
 find_package(OpenGL REQUIRED)
 find_package(GLUT REQUIRED)
 find_package(GLEW REQUIRED)
 if (UNIX)
  find_package(X11 REQUIRED)
 endif()
endif()

# =========================
# Frontend
# =========================
add_subdirectory(Frontend)

# =========================
# Backend
# =========================
add_subdirectory(Backend)

# =========================
# App
# =========================
add_subdirectory(App)
