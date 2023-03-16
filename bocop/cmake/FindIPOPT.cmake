set(BOCOP_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

#  ${BOCOP_ROOT_PATH}/Library would be the path for conda install of ipopt on windows. 
# Disabled for now since package is too old, replaced with manual install at ipopt/
set(TARGET_PATH ${BOCOP_ROOT_PATH}) 

# old windows libs require additional libs as well
if(NOT UNIX)

	set(TARGET_PATH_IPOPT 		${BOCOP_ROOT_PATH}) # This finds IPOPT lib from the Conda install. Still this release uses dmumps as "dmumps.dll" and not "libdmumps.dll" and I don't know how to do different than renaming the Conda DLL.
	set(TARGET_PATH_IPOPT_H 	${BOCOP_ROOT_PATH}) 

	find_library(IPOPT_LIBRARIES libipopt PATHS ${TARGET_PATH_IPOPT} PATH_SUFFIXES "dll" REQUIRED)
	find_path(IPOPT_INCLUDE_DIR IpoptConfig.h PATHS ${TARGET_PATH_IPOPT_H} PATH_SUFFIXES "include/coin" REQUIRED)

	# set(TARGET_PATH_IPOPT_DEPS  ${BOCOP_ROOT_PATH}/thirdparty/coinor-1.8.0) # /thirdparty/coinor-1.8.0 This finds MUMPS & Friends from COIN-OR release.
	# find_library(MUMPS_LIBRARY 	coinmumps PATHS ${TARGET_PATH_IPOPT_DEPS} PATH_SUFFIXES "lib" REQUIRED)
	# find_library(BLAS_LIBRARY 	coinblas PATHS ${TARGET_PATH_IPOPT_DEPS} PATH_SUFFIXES "lib" REQUIRED)
	# find_library(LAPACK_LIBRARY coinlapack PATHS ${TARGET_PATH_IPOPT_DEPS} PATH_SUFFIXES "lib" REQUIRED)
	set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${MUMPS_LIBRARY_DLL} ${MUMPS_LIBRARY} ${LAPACK_LIBRARY} ${BLAS_LIBRARY}) #order lapack/blas matters -_-
else()
	# installed by conda in bocop folder
	set(TARGET_PATH ${BOCOP_ROOT_PATH}) 

	find_library(IPOPT_LIBRARIES ipopt PATHS ${TARGET_PATH_IPOPT} PATH_SUFFIXES "lib" REQUIRED)
	find_path(IPOPT_INCLUDE_DIR IpoptConfig.h PATHS ${TARGET_PATH_IPOPT} PATH_SUFFIXES "include/coin" REQUIRED)
endif()