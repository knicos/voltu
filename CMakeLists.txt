cmake_minimum_required (VERSION 3.1.0)
include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
include(CheckLanguage)

project (ftl.utu.fi)

include(CTest)
enable_testing()

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package( glog REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( LibSGM )

include_directories(${PROJECT_SOURCE_DIR}/common/cpp/include)

check_language(CUDA)
if (CUDA_FOUND)
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "-g -DDEBUG -D_DEBUG -Wall")
set(CMAKE_CUDA_FLAGS_RELEASE "")
add_definitions(-DHAVE_CUDA)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif (CUDA_FOUND)

include(git_version)

if (WIN32)
	file(TO_CMAKE_PATH "$ENV{USERPROFILE}" ENV_USERPROFILE)
	set(FTL_LOCAL_CONFIG_ROOT "\"${ENV_USERPROFILE}/AppData/ftl\"")
	message(STATUS "Config root is ${FTL_LOCAL_CONFIG_ROOT}")
	set(FTL_LOCAL_CACHE_ROOT "\"$ENV{USERPROFILE}/AppData/ftl\"")
	set(FTL_LOCAL_DATA_ROOT "\"$ENV{USERPROFILE}/AppData/ftl\"")
	add_definitions(-DWIN32)

	set(CMAKE_CXX_FLAGS "/std:c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif (WIN32)

if (UNIX)
	add_definitions(-DUNIX)
	
	set(FTL_LOCAL_CONFIG_ROOT "\"$ENV{HOME}/.config/ftl\"")
	message(STATUS "Config root is ${FTL_LOCAL_CONFIG_ROOT}")
	set(FTL_LOCAL_CACHE_ROOT "\"$ENV{HOME}/.cache/ftl\"")
	set(FTL_LOCAL_DATA_ROOT "\"$ENV{HOME}/.local/share/ftl\"")
	
	set(FTL_GLOBAL_CONFIG_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")
	message(STATUS "Global config root is ${FTL_GLOBAL_CONFIG_ROOT}")
	set(FTL_GLOBAL_CACHE_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")
	set(FTL_GLOBAL_DATA_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")

	set(CMAKE_CXX_FLAGS "-std=c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -pg -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -msse3 -mfpmath=sse")
endif (UNIX)

SET(CMAKE_USE_RELATIVE_PATHS ON)

add_subdirectory(net/cpp)
add_subdirectory(cv-node)

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h.in
               ${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h
)

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp.in
               ${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp
)
