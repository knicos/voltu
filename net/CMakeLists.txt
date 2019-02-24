cmake_minimum_required (VERSION 2.8.11)

include (CheckIncludeFile)
include (CheckFunctionExists)
project (ftlnet)

#find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/includes)
include_directories(${PROJECT_BINARY_DIR})

set(ftl_VERSION_MAJOR "1")
set(ftl_VERSION_MINOR "0")
set(ftl_VERSION_PATCH "0")

set(CMAKE_CXX_FLAGS "-pthread -fopenmp -std=c++17 -Wall -Wno-deprecated -Werror -Wno-psabi")
set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -pg -Wall -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

SET(CMAKE_USE_RELATIVE_PATHS ON)

set(FTLSOURCE
	src/net.cpp
	src/listener.cpp
	src/socket.cpp
	src/dispatcher.cpp
)

check_include_file("uriparser/Uri.h" HAVE_URI_H)
if(NOT HAVE_URI_H)
	message(FATAL_ERROR "Uriparser not found")
endif()
check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

add_library(ftlnet ${FTLSOURCE})
target_include_directories(ftlnet PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlnet pthread)

ADD_SUBDIRECTORY(test)
