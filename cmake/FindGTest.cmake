# - Try to find GTest
#
# The following variables are optionally searched for defaults
#  GTEST_ROOT_DIR:            Base directory where all GTEST components are found
#
# The following are set after configuration is done: 
#  GTEST_FOUND
#  GTEST_INCLUDE_DIRS
#  GTEST_LIBRARIES

include(FindPackageHandleStandardArgs)

# GTest should be installed at /usr/local
set(GTEST_ROOT_DIR /usr/local
 CACHE PATH "Folder contains Google gtest"
)

find_path(GTEST_INCLUDE_DIR gtest/gtest.h
          PATHS ${GTEST_ROOT_DIR}/include)

find_library(GTEST_LIBRARY gtest
             PATHS ${GTEST_ROOT_DIR}
             PATH_SUFFIXES
               lib
               lib64)

find_package_handle_standard_args(GTEST
  DEFAULT_MSG
  GTEST_INCLUDE_DIR
  GTEST_LIBRARY
)

if(GTEST_FOUND)
  set(GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIR})
  set(GTEST_LIBRARIES ${GTEST_LIBRARY})
  message(STATUS "Found GTEST:")
  message(STATUS "GTEST_INCLUDE_DIRS=${GTEST_INCLUDE_DIRS}")
  message(STATUS "GTEST_LIBRARIES   =${GTEST_LIBRARIES}")
else()
  message(STATUS "GTEST is not found")
endif()