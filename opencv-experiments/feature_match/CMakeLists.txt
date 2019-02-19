cmake_minimum_required (VERSION 2.8.11)
project (ftl-exp)

find_package( OpenCV REQUIRED )
#find_package(PkgConfig)
#pkg_check_modules(GTKMM gtkmm-3.0)

# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/includes)
include_directories(${PROJECT_BINARY_DIR})

set(ftl_VERSION_MAJOR "1")
set(ftl_VERSION_MINOR "0")
set(ftl_VERSION_PATCH "0")

set(CMAKE_CXX_FLAGS "-pthread -std=c++14 -Wall -Wno-deprecated -Werror -Wno-psabi")
set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -pg -Wall -Werror")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

SET(CMAKE_USE_RELATIVE_PATHS ON)

set(FTLSOURCE
	src/main.cpp
)

add_executable(ftl-exp ${FTLSOURCE})
target_include_directories(ftl-exp PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-exp pthread ${OpenCV_LIBS})


