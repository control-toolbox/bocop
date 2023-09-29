# Sources
set(INCLUDE_DIR AD DOCP NLP OCP tools)
include_directories(${INCLUDE_DIR})

set(BOCOP_HEADERS
  AD/dOCPCppAD.h
  DOCP/dOCP.h
  DOCP/dODE.h
  DOCP/dControl.h
  DOCP/dState.h
  DOCP/solution.h
  NLP/NLP.h
  NLP/NLPSolver.h
  NLP/NLPSolverIpopt.h
  OCP/OCP.h
  tools/bufferAdaptor.h
  tools/tools.h
  tools/tools_interpolation.h
  )

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

set(CPP_FILE "problem.cpp" CACHE STRING "Problem cpp file") # can be overriden at cmake call
file(TO_CMAKE_PATH "${PROBLEM_DIR}" PROBLEM_DIR)
set(PROBLEM_SOURCES ${PROBLEM_DIR}/${CPP_FILE})


# Default build type (RelWithDebInfo)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
else()
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

# core library +++ currently rebuilt every time if the build/ folder is deleted -_- (putting the build above does not change this)
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")
add_library(bocopcore${SUFFIX} SHARED ${BOCOP_SOURCES} ${BOCOP_HEADERS})
target_link_libraries(bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
set_target_properties(bocopcore PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Build core library and executable for specified problem
# +++ distinguish between wrapper and exec ?
if(PROBLEM_DIR)
  set(EXECUTABLE_OUTPUT_PATH ${PROBLEM_DIR})
  add_executable(bocopproblem ${BOCOP_HEADERS} ${PROBLEM_SOURCES} main.cpp)
  target_link_libraries(bocopproblem bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
endif(PROBLEM_DIR)

# CMakeLists.txt ends here
