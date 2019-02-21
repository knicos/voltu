cmake_minimum_required (VERSION 2.8.11)

include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
project (ftlp2prm)

#find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR})

set(ftl_VERSION_MAJOR "1")
set(ftl_VERSION_MINOR "0")
set(ftl_VERSION_PATCH "0")

set(CMAKE_CXX_FLAGS "-pthread -fopenmp -std=c++17 -Wall -Wno-deprecated -Werror -Wno-psabi")
set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -pg -Wall -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

SET(CMAKE_USE_RELATIVE_PATHS ON)

set(FTLSOURCE
	src/cluster.cpp
	src/p2prm.cpp
)
add_library(ftl-p2prm ${FTLSOURCE})

check_include_file("uriparser/Uri.h" HAVE_URI_H)
if(NOT HAVE_URI_H)
	message(FATAL_ERROR "Uriparser is required")
endif()
check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

# Look for FTL Net
find_path(FTL_NET "ftl/net.hpp")
if (NOT FTL_NET)
	message(STATUS "FTL Net is not installed")
	find_path(FTL_NET "ftl/net.hpp" PATHS ../net/include)
	if (NOT FTL_NET)
		message(FATAL_ERROR "FTL Net is required")
	endif()
	#TODO Ensure this is built
	message(STATUS ${FTL_NET})
	include_directories(${FTL_NET})
endif()

target_include_directories(ftl-p2prm PUBLIC ${PROJECT_SOURCE_DIR}/include)
#target_link_libraries(libftlp2pra pthread)

ADD_SUBDIRECTORY(test)
