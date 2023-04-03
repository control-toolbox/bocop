# Sources
set(INC_DIR AD DOCP NLP OCP tools)
include_directories(${INC_DIR})

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

if(NOT NOWRAPPER)
  list(INSERT WRAPPER_INCLUDES 0 ${INC_DIR})
  list(INSERT WRAPPER_INCLUDES 0 ${DTK_SWIG_INCLUDE_PATH} ${Python3_INCLUDE_DIRS})
endif()

# Default build type (RelWithDebInfo)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
else()
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

# Code coverage
message(STATUS "Coverage option: ${COVERAGE}")
if(COVERAGE)
  include(CodeCoverage)
  append_coverage_compiler_flags()
endif()

# Build setup
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(APPLE)
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -flat_namespace")
endif(APPLE)

if(NOT NOWRAPPER)
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i PROPERTIES CPLUSPLUS ON ${DOCSTRING_DEPS})
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i PROPERTIES INCLUDE_DIRECTORIES "${WRAPPER_INCLUDES}")
  set(SWIG_MODULE_NAME bocopwrapper)
  list(APPEND CMAKE_SWIG_FLAGS -v -O -python -py3 -shadow -c++)
endif()

# Build core library and executable for specified problem
# +++ distinguish between wrapper and exec ?
if(PROBLEM_DIR)
  set(LIBRARY_OUTPUT_PATH ${PROBLEM_DIR})
  set(EXECUTABLE_OUTPUT_PATH ${PROBLEM_DIR})

  add_library(bocop${SUFFIX} SHARED ${BOCOP_SOURCES} ${BOCOP_HEADERS} ${PROBLEM_SOURCES})
  target_link_libraries(bocop${SUFFIX} ${IPOPT_LIBRARIES})

  if(NOT NOWRAPPER)
    swig_add_library(bocopwrapper${SUFFIX} LANGUAGE python SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i)
    swig_link_libraries(bocopwrapper${SUFFIX} ${Python3_LIBRARIES} bocop${SUFFIX})
    add_custom_command(TARGET bocopwrapper${SUFFIX} POST_BUILD
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bocopwrapper.i
      COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different ${CMAKE_BINARY_DIR}/src/bocopwrapper.py  ${PROBLEM_DIR}
      COMMENT "-- Moving python modules to ${PROBLEM_DIR}")
  endif()

  add_executable(bocopApp ${BOCOP_SOURCES} ${BOCOP_HEADERS} ${PROBLEM_SOURCES} main.cpp)
  target_link_libraries(bocopApp ${IPOPT_LIBRARIES})
endif(PROBLEM_DIR)

# CMakeLists.txt ends here
