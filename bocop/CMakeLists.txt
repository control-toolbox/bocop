# BOCOP
cmake_minimum_required(VERSION 3.16)
project(bocop)
file(READ "VERSION" ver)
set(${PROJECT_NAME}_VERSION ver)

# some general settings
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Default build type (RelWithDebInfo)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
else()
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

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

#add_subdirectory(src)
# set core bocop library as external project
include(ExternalProject)
ExternalProject_Add(bocopcorelib 
                    SOURCE_DIR ${CMAKE_SOURCE_DIR}/src/
                    DOWNLOAD_COMMAND ""
                    BINARY_DIR ${CMAKE_SOURCE_DIR}/build/
                    BUILD_ALWAYS TRUE
                    INSTALL_COMMAND ""
                    )

# specific executable for given problem
message(STATUS "Build executable for problem at ${PROBLEM_DIR}")
message(STATUS "bocop headers ${BOCOP_HEADERS}")
set(CPP_FILE "problem.cpp" CACHE STRING "Problem cpp file") # can be overriden at cmake call
file(TO_CMAKE_PATH "${PROBLEM_DIR}" PROBLEM_DIR)
set(EXECUTABLE_OUTPUT_PATH ${PROBLEM_DIR})
add_executable(bocopproblem ${BOCOP_HEADERS} ${PROBLEM_DIR}/${CPP_FILE} src/main.cpp)
add_dependencies(bocopproblem bocopcorelib)
#target_include_directories(totoexec PUBLIC src/src1/)
target_link_libraries(bocopproblem bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
target_link_directories(bocopproblem PUBLIC ${CMAKE_SOURCE_DIR}/lib/) #find_library may be more robust...

#### CMakeLists.txt ends here
