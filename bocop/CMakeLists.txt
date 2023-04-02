# BOCOP
cmake_minimum_required(VERSION 3.16)
project(bocop)
file(READ "VERSION" ver)
set(${PROJECT_NAME}_VERSION ver)

message(STATUS "Problem path: ${PROBLEM_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

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

# swig
find_package(SWIG 4 REQUIRED)
include(${SWIG_USE_FILE})

add_subdirectory(src)
#### CMakeLists.txt ends here
