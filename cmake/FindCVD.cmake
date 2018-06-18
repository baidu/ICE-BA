# - Try to find CVD
#
# The following variables are optionally searched for defaults
#  CVD_ROOT_DIR:  Base directory where all CVD components are found
#
# The following are set after configuration is done:
#  CVD_FOUND
#  CVD_INCLUDE_DIRS
#  CVD_LIBRARIES

include(FindPackageHandleStandardArgs)

set(CVD_ROOT_DIR /usr/local/
 CACHE PATH "Folder contains libcvd"
)

find_path(CVD_INCLUDE_DIR cvd/config.h
          PATHS ${CVD_ROOT_DIR}/include)

find_library(CVD_LIBRARY cvd
             PATHS ${CVD_ROOT_DIR}
             PATH_SUFFIXES
               lib
               lib64)

find_package_handle_standard_args(CVD
  DEFAULT_MSG
  CVD_INCLUDE_DIR
  CVD_LIBRARY
)

if(CVD_FOUND)
  set(CVD_INCLUDE_DIRS ${CVD_INCLUDE_DIR})
  set(CVD_LIBRARIES ${CVD_LIBRARY})
  message(STATUS "Found CVD:")
  message(STATUS "CVD_INCLUDE_DIRS=${CVD_INCLUDE_DIRS}")
  message(STATUS "CVD_LIBRARIES   =${CVD_LIBRARIES}")
else()
  message(STATUS "CVD is not found")
endif()