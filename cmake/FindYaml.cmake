# - Try to find Yaml
#
# The following variables are optionally searched for defaults
#  YAMLCPP_ROOT_DIR:            Base directory where all YAMLCPP components are found
#
# The following are set after configuration is done: 
#  YAMLCPP_FOUND
#  YAMLCPP_INCLUDE_DIRS
#  YAMLCPP_LIBRARIES

include(FindPackageHandleStandardArgs)

set(YAMLCPP_ROOT_DIR /usr/local/
 CACHE PATH "Folder contains yaml-cpp"
)

find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
          PATHS ${YAMLCPP_ROOT_DIR}/include)

find_library(YAMLCPP_LIBRARY yaml-cpp
             PATHS ${YAMLCPP_ROOT_DIR}
             PATH_SUFFIXES
               lib
               lib64)

find_package_handle_standard_args(YAMLCPP
  DEFAULT_MSG
  YAMLCPP_INCLUDE_DIR
  YAMLCPP_LIBRARY
)

if(YAMLCPP_FOUND)
  set(YAMLCPP_INCLUDE_DIRS ${YAMLCPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAMLCPP_LIBRARY})
  message(STATUS "Found YAMLCPP:")
  message(STATUS "YAMLCPP_INCLUDE_DIRS=${YAMLCPP_INCLUDE_DIRS}")
  message(STATUS "YAMLCPP_LIBRARIES   =${YAMLCPP_LIBRARIES}")
else()
  message(STATUS "YAMLCPP is not found")
endif()