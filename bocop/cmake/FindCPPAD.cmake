set(BOCOP_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")


set(TARGET_PATH ${BOCOP_ROOT_PATH})
# search include (no libs)
find_path(CPPAD_INCLUDE_DIR cppad.hpp PATHS ${TARGET_PATH} PATH_SUFFIXES "include/cppad" REQUIRED)
set(CPPAD_INCLUDE_DIR ${CPPAD_INCLUDE_DIR}/..)
