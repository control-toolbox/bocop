# BOCOP CORE LIB
cmake_minimum_required(VERSION 3.16)
project(bocopcorelib)

########################################################################
# FIND THIRD PARTY LIBRAIRIES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake")

# ipopt (include and lib)
find_package(IPOPT REQUIRED)
include_directories(${IPOPT_INCLUDE_DIR})
message(STATUS "Using IPOPT found at " ${IPOPT_LIBRARIES})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) # +++ to find the dylib on mac ?

########################################################################
# BUILD SETTINGS
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
if(APPLE)
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -undefined dynamic_lookup -flat_namespace")
endif()

# default build type (RelWithDebInfo)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
else()
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

# code coverage
message(STATUS "Coverage option: ${COVERAGE}")
if(COVERAGE)
  include(CodeCoverage)
  append_coverage_compiler_flags()
endif()

# core includes and sources
set(BOCOP_INCLUDE_DIR AD DOCP NLP OCP tools)
include_directories(${BOCOP_INCLUDE_DIR})

set(BOCOP_SOURCES
  AD/dOCPCppAD.cpp
  DOCP/dOCP.cpp
  DOCP/dODE.cpp
  DOCP/dControl.cpp
  DOCP/dState.cpp
  DOCP/solution.cpp
  NLP/NLPSolverIpopt.cpp
  OCP/OCP.cpp
  tools/tools.cpp
  tools/tools_interpolation.cpp
  )

# core library
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/../lib")
add_library(bocopcore${SUFFIX} SHARED ${BOCOP_SOURCES})
target_link_libraries(bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
# add some install step here so that the lib is better recognized toplevel ?

# CMakeLists.txt ends here
