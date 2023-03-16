## ###################################################################
## BOCOP
## ###################################################################

cmake_minimum_required(VERSION 3.16)

project(bocop)

## ###################################################################
## Version setup
## ###################################################################
file(READ "VERSION" ver)
set(${PROJECT_NAME}_VERSION ver)

## ###################################################################
## problem folder REQUIRED
## ###################################################################
message(STATUS "Problem path: ${PROBLEM_DIR}")

## ###################################################################
## cmake modules folder
## ###################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

## #################################################################
## Build setup
## #################################################################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

## #################################################################
## Dependencies
## #################################################################



# cppad (no lib, only headers)
find_package(CPPAD REQUIRED)
message(STATUS "Using CPPAD found at " ${CPPAD_INCLUDE_DIR})
include_directories(${CPPAD_INCLUDE_DIR})


# ipopt (include and lib)
find_package(IPOPT REQUIRED)
include_directories(${IPOPT_INCLUDE_DIR})
message(STATUS "Using IPOPT found at " ${IPOPT_LIBRARIES})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) # to find the dylib on mac


# Must find Python3 before IPOPT, probably because FindIPOPT sets LIBRARY_PREFIXES to "lib" and "libpython" doesn't exist on Windows
set(Python3_FIND_VIRTUALENV ONLY)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED) # not found on Windows



include_directories(${Python3_INCLUDE_DIRS})



find_package(SWIG 4 REQUIRED)
include(${SWIG_USE_FILE})



## ###################################################################
## Input
## ###################################################################
add_subdirectory(src)


## ###################################################################
## Install
## ###################################################################

# use always full RPATH (libs will be found regardless of LD_LIBRARY_PATH and install location
#set(CMAKE_SKIP_BUILD_RPATH  FALSE)
#set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
#list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
#if("${isSystemDir}" STREQUAL "-1")
#  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#endif("${isSystemDir}" STREQUAL "-1")

## USE THIS FOR CONDA BUILD (for cpack test and set prefix ?)
#set(CMAKE_INSTALL_PREFIX $ENV{PREFIX})

## BOCOP SOURCE AND SCRIPTS
#install(FILES ${PROJECT_SOURCE_DIR}/CMakeLists.txt DESTINATION .)
#install(FILES ${PROJECT_SOURCE_DIR}/README.md DESTINATION .)
#install(FILES ${PROJECT_SOURCE_DIR}/setup.py DESTINATION .)
#install(FILES ${PROJECT_SOURCE_DIR}/TODO.md DESTINATION .)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/bocop DESTINATION . USE_SOURCE_PERMISSIONS)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/cmake DESTINATION . USE_SOURCE_PERMISSIONS)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/matlab DESTINATION . USE_SOURCE_PERMISSIONS)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/packages DESTINATION . USE_SOURCE_PERMISSIONS)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/src DESTINATION . USE_SOURCE_PERMISSIONS)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/test DESTINATION . USE_SOURCE_PERMISSIONS)

## EXAMPLES: install all files in problem folder except build/, executable, backup files and result.out
#set(EXAMPLES_LIST
    #beam
  #fuller
  #goddard
  #vanderpol
  #)
#FOREACH(example ${EXAMPLES_LIST})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/../examples/${example}/
        #DESTINATION examples/${example} USE_SOURCE_PERMISSIONS
                #PATTERN "build" EXCLUDE PATTERN "bocop.exe" EXCLUDE PATTERN "bocop"
                #EXCLUDE PATTERN "result.out" EXCLUDE PATTERN "*.backup" EXCLUDE)
#ENDFOREACH()

## THIRDPARTY

## manual install for cppad/ipopt on windows for use with MinGW (until we have proper native VS conda packages...)
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty DESTINATION . USE_SOURCE_PERMISSIONS)

#######################################################################
#### CMakeLists.txt ends here
