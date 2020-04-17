cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(libstereo)

option(BUILD_MIDDLEBURY     "Build Middlebury evaluation" OFF)
option(BUILD_TESTS          "Build unit tests" ON)
option(LIBSTEREO_SHARED     "Build a shared library" OFF)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package( Threads REQUIRED )
#find_package(CUDA REQUIRED)

enable_language(CUDA)

set(CMAKE_CXX_CPPCHECK "cppcheck")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_USE_RELATIVE_PATHS ON)
set(CMAKE_CXX_FLAGS_RELEASE)

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CUDA_FLAGS "--gpu-architecture=compute_61 -std=c++14 -Xcompiler -fPIC -Xcompiler ${OpenMP_CXX_FLAGS}")
    set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

if (LIBSTEREO_SHARED)
    add_library(libstereo SHARED
                src/stereo_gradientstree.cu
                src/stereo_misgm.cu
                src/stereo_censussgm.cu
                src/stereo_sgmp.cpp
                src/costs/census.cu
                src/costs/gradient.cu
                src/costs/mutual_information.cu
                src/median_filter.cu
                src/median_filter_fixstars.cu
    )
set_target_properties(libstereo PROPERTIES PUBLIC_HEADER include/stereo.hpp)

else()
    add_library(libstereo
                src/stereo_gradientstree.cu
                src/stereo_misgm.cu
                src/stereo_censussgm.cu
                src/stereo_adcensussgm.cu
                src/stereo_adsgm.cu
                #src/stereo_sgmp.cpp
                src/costs/census.cu
                src/costs/gradient.cu
                src/costs/mutual_information.cu
                src/median_filter.cu
                src/median_filter_fixstars.cu
    )
endif()

if (USE_GPU)
    target_compile_definitions(libstereo PUBLIC USE_GPU)
endif()

target_include_directories(libstereo PRIVATE src/ include/)
target_include_directories(libstereo PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(libstereo Threads::Threads ${OpenCV_LIBS} ${CUDA_LIBRARIES})

if (BUILD_MIDDLEBURY)
    add_subdirectory(middlebury/)
endif()

if (BUILD_TESTS)
    add_subdirectory(test)
endif()
