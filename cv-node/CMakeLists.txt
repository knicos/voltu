cmake_minimum_required (VERSION 3.1.0)
include (CheckIncludeFile)
include (CheckFunctionExists)

project (cv-node)

include(CTest)
enable_testing()

set(THREADS_PREFER_PTHREAD_FLAG ON)

if (WIN32)
find_package( glog REQUIRED )
include_directories(${glog_DIR})
endif (WIN32)
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
#find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR})

set(ftl_VERSION_MAJOR "1")
set(ftl_VERSION_MINOR "0")
set(ftl_VERSION_PATCH "0")
set(FTL_VERSION "${ftl_VERSION_MAJOR}.${ftl_VERSION_MINOR}.${ftl_VERSION_PATCH}")
add_definitions(-DFTL_VERSION=${FTL_VERSION})

if (WIN32)
	file(TO_CMAKE_PATH "$ENV{USERPROFILE}" ENV_USERPROFILE)
	set(FTL_CONFIG_ROOT "\"${ENV_USERPROFILE}/AppData/ftl\"")
	message(STATUS "Config root is ${FTL_CONFIG_ROOT}")
	set(FTL_CACHE_ROOT "$ENV{USERPROFILE}/AppData/ftl")
	set(FTL_DATA_ROOT "$ENV{USERPROFILE}/AppData/ftl")
	add_definitions(-DWIN32)
endif (WIN32)

if (UNIX)
	add_definitions(-DUNIX)
	set(FTL_CONFIG_ROOT "\"$ENV{HOME}/.config/ftl\"")
	message(STATUS "Config root is ${FTL_CONFIG_ROOT}")
	set(FTL_CACHE_ROOT "$ENV{HOME}/.cache/ftl")
	set(FTL_DATA_ROOT "$ENV{HOME}/.local/share/ftl")
endif (UNIX)

add_definitions("-DFTL_CONFIG_ROOT=${FTL_CONFIG_ROOT}")
add_definitions(-DFTL_CACHE_ROOT=${FTL_CACHE_ROOT})
add_definitions(-DFTL_DATA_ROOT=${FTL_DATA_ROOT})

set(CMAKE_CXX_FLAGS "-pthread -std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -pg -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -msse3 -mfpmath=sse")

SET(CMAKE_USE_RELATIVE_PATHS ON)

set(CVNODESRC
	src/main.cpp
	src/calibrate.cpp
	src/local.cpp
	src/sync.cpp
	src/disparity.cpp
	src/algorithms/rtcensus.cpp
	src/algorithms/opencv_sgbm.cpp
)

add_executable(cv-node ${CVNODESRC})
target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(cv-node Threads::Threads ${OpenCV_LIBS} glog)


