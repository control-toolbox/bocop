cmake_minimum_required(VERSION 3.16)
project(bocopcorelib)

set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")
message("Build Bocop core lib at ${LIBRARY_OUTPUT_PATH}")

# Sources
set(INCLUDE_DIR AD DOCP NLP OCP tools)
include_directories(${INCLUDE_DIR})

# core headers
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

# core sources
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
add_library(bocopcore${SUFFIX} SHARED ${BOCOP_SOURCES} ${BOCOP_HEADERS})
target_link_libraries(bocopcore${SUFFIX} ${IPOPT_LIBRARIES})

# CMakeLists.txt ends here
