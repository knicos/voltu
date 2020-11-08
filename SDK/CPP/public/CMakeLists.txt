cmake_minimum_required (VERSION 3.16.0)

project (voltu_sdk VERSION 0.0.1)

include(GNUInstallDirs)

option(WITH_OPENCV "Build with OpenCV wrapper" ON)
option(WITH_PYTHON "Build Python module" OFF)

find_package( Eigen3 REQUIRED NO_MODULE )
find_package( Threads REQUIRED )

if (WITH_OPENCV)
	find_package( OpenCV REQUIRED )
endif()

if(WIN32)
	add_definitions(-DWIN32)
	set(CMAKE_GENERATOR_TOOLSET "host=x64")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2 /MP4 /std:c++14 /wd4996")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DFTL_DEBUG /Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
	set(OS_LIBS "")
else()
	add_definitions(-DUNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -march=native -mfpmath=sse -Wall -Werror=unused-result -Werror=return-type -pthread")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
	set(OS_LIBS "dl")
endif()

set(VOLTU_SRCS
	voltu.cpp
)

set(OPTIONAL_DEPENDENCIES)

if (WITH_OPENCV)
	list(APPEND OPTIONAL_DEPENDENCIES ${OpenCV_LIBS})
	list(APPEND VOLTU_SRCS voltu_cv.cpp)
	set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
endif()

add_library(voltu_sdk STATIC ${VOLTU_SRCS})

target_include_directories(voltu_sdk
	PUBLIC include)
target_link_libraries(voltu_sdk ${OS_LIBS} Threads::Threads ${OPTIONAL_DEPENDENCIES} Eigen3::Eigen)

add_executable(voltu_basic_test
	samples/basic_test/main.cpp
)
target_link_libraries(voltu_basic_test voltu_sdk)

add_executable(voltu_basic_file
	samples/basic_file/main.cpp
)
target_link_libraries(voltu_basic_file voltu_sdk)

add_executable(voltu_basic_virtual_cam
	samples/basic_virtual_cam/main.cpp
)
target_link_libraries(voltu_basic_virtual_cam voltu_sdk)

add_executable(voltu_fusion_evaluator
	samples/fusion_evaluator/main.cpp
)
target_link_libraries(voltu_fusion_evaluator voltu_sdk)

if (WITH_PYTHON)
	add_subdirectory(ext/pybind11)
	add_subdirectory(python)
endif()
