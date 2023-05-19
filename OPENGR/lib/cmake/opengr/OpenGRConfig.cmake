# - Config file for the OpenGR package
# It defines the following variables
#  OPENGR_INCLUDE_DIRS - include directories for OpenGR
#  OPENGR_LIB_DIR      - libraries to link against
#  OPENGR_LIBRARIES    - libraries to link against
#
#
# To use
# ::
# find_package(OpenGR REQUIRED)
# include_directories(${OpenGR_INCLUDE_DIR})
# add_executable(foo foo.cc)
# target_link_libraries(foo ${OpenGR_LIBRARIES})
# link_directories(${OpenGR_LIB_DIR})
#
# find_package( Eigen3 REQUIRED )
# include_directories( ${EIGEN3_INCLUDE_DIR} )



####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

if(OFF)
  include("${CMAKE_CURRENT_LIST_DIR}/chealpixTargets.cmake")
endif(OFF)


include("${CMAKE_CURRENT_LIST_DIR}/OpenGR-AccelLibTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/OpenGR-AlgoLibTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/OpenGR-UtilsLibTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/OpenGRConfigVersion.cmake")


# Compute paths
set(OpenGR_INCLUDE_DIR "D:/OpenGR-master/OpenGR-master/install/include/")
set(OpenGR_LIB_DIR "D:/OpenGR-master/OpenGR-master/install/lib/")
set(OpenGR_LIBRARIES gr::algo gr::utils gr::accel)

if(OFF)
  set(OpenGR_LIBRARIES chealpix)
endif(OFF)

set(OpenGR_FOUND 1)
