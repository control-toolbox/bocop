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

# core library +++ currently rebuilt every time if the build/ folder is deleted -_- (putting the build above does not change this)
message("build lib")
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")
add_library(bocopcore${SUFFIX} SHARED ${BOCOP_SOURCES} ${BOCOP_HEADERS})
target_link_libraries(bocopcore${SUFFIX} ${IPOPT_LIBRARIES})
#set_target_properties(bocopcore PROPERTIES EXCLUDE_FROM_ALL TRUE)

# problem exec
message("build exec")
set(CPP_FILE "problem.cpp" CACHE STRING "Problem cpp file") # can be overriden at cmake call
file(TO_CMAKE_PATH "${PROBLEM_DIR}" PROBLEM_DIR)
set(EXECUTABLE_OUTPUT_PATH ${PROBLEM_DIR})
add_executable(bocopproblem ${BOCOP_HEADERS} ${PROBLEM_DIR}/${CPP_FILE} main.cpp)
target_link_libraries(bocopproblem bocopcore${SUFFIX} ${IPOPT_LIBRARIES})

# CMakeLists.txt ends here
