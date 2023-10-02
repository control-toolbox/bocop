########################################################################
# BOCOP PROBLEM EXECUTABLE
cmake_minimum_required(VERSION 3.16)
project(bocop)
file(READ "VERSION" ver)
set(${PROJECT_NAME}_VERSION ver)

########################################################################
# FIND THIRD PARTY LIBRAIRIES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# cppad (no lib, only headers)
find_package(CPPAD REQUIRED)
message(STATUS "Using CPPAD found at " ${CPPAD_INCLUDE_DIR})
include_directories(${CPPAD_INCLUDE_DIR})

# ipopt (include and lib)
find_package(IPOPT REQUIRED)
include_directories(${IPOPT_INCLUDE_DIR})
message(STATUS "Using IPOPT found at " ${IPOPT_LIBRARIES})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) # to find the dylib on mac

# swig for python wrapper
if(NOT NOWRAPPER)
  # must find Python3 before IPOPT, probably because FindIPOPT sets 
  # LIBRARY_PREFIXES to "lib" and "libpython" doesn't exist on Windows
  set(Python3_FIND_VIRTUALENV ONLY)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED) # not found on Windows
  include_directories(${Python3_INCLUDE_DIRS})

  find_package(SWIG 4 REQUIRED)
  include(${SWIG_USE_FILE})
endif()

########################################################################
# BUILD SETTINGS
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
if(APPLE)
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -flat_namespace")
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

########################################################################
# BUILD BOCOP CORE LIBRARY
include(ExternalProject)
ExternalProject_Add(bocopcorelib 
                    SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/
                    DOWNLOAD_COMMAND ""
                    BINARY_DIR ${CMAKE_SOURCE_DIR}/build/
                    BUILD_ALWAYS TRUE
                    INSTALL_COMMAND ""
                    )

########################################################################
# BUILD SPECIFIC PROBLEM

# command line executable
#if(EXEC)
message(STATUS "Build executable for problem at ${PROBLEM_DIR}")
set(BOCOP_INCLUDE_DIR src/AD src/DOCP src/NLP src/OCP src/tools)
include_directories(${BOCOP_INCLUDE_DIR})
set(CPP_FILE "problem.cpp" CACHE STRING "Problem cpp file") # can be overriden at cmake call
file(TO_CMAKE_PATH "${PROBLEM_DIR}" PROBLEM_DIR)
set(EXECUTABLE_OUTPUT_PATH ${PROBLEM_DIR})
add_executable(bocopproblem ${PROBLEM_DIR}/${CPP_FILE} src/main.cpp)
add_dependencies(bocopproblem bocopcorelib)
target_link_libraries(bocopproblem bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
target_link_directories(bocopproblem PUBLIC ${CMAKE_SOURCE_DIR}/lib/) # NB. find_library may be more robust...

# python wrapper # need to add problem.cpp since not in bocopcore lib unlike previous bocop lib ?
if(NOT NOWRAPPER)

  list(INSERT WRAPPER_INCLUDES 0 ${BOCOP_INCLUDE_DIR})
  list(INSERT WRAPPER_INCLUDES 0 ${DTK_SWIG_INCLUDE_PATH} ${Python3_INCLUDE_DIRS})

  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i PROPERTIES CPLUSPLUS ON ${DOCSTRING_DEPS})
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i PROPERTIES INCLUDE_DIRECTORIES "${WRAPPER_INCLUDES}")
  set(SWIG_MODULE_NAME bocopwrapper)
  list(APPEND CMAKE_SWIG_FLAGS -v -O -python -py3 -shadow -c++)

  swig_add_library(bocopwrapper${SUFFIX} LANGUAGE python SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i)
  swig_link_libraries(bocopwrapper${SUFFIX} ${Python3_LIBRARIES} bocopcore${SUFFIX})
  add_custom_command(TARGET bocopwrapper${SUFFIX} POST_BUILD
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i
    COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${CMAKE_BINARY_DIR}/src/bocopwrapper.py  ${PROBLEM_DIR}
    COMMENT "-- Moving python modules to ${PROBLEM_DIR}")
endif()


#### CMakeLists.txt ends here
