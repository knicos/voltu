cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
include(CheckLanguage)

project(libstereo)

option(BUILD_MIDDLEBURY     "Build Middlebury evaluation" OFF)
option(BUILD_TESTS          "Build unit tests" ON)
option(LIBSTEREO_SHARED     "Build a shared library" OFF)
option(USE_GPU              "Use the GPU instead of CPU" ON)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package( Threads REQUIRED )
#find_package(CUDA REQUIRED)

check_language(CUDA)
enable_language(CUDA)

set(CMAKE_CXX_CPPCHECK "cppcheck")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_USE_RELATIVE_PATHS ON)
set(CMAKE_CXX_FLAGS_RELEASE)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CUDA_HOST_COMPILER gcc-7)
    set(CMAKE_CUDA_FLAGS "--gpu-architecture=compute_61 -std=c++14 -Xcompiler -fPIC -Xcompiler ${OpenMP_CXX_FLAGS} --expt-relaxed-constexpr")
    set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
else()
    set(CMAKE_CUDA_FLAGS "--gpu-architecture=compute_61 -std=c++14 --expt-relaxed-constexpr")
    set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

if (LIBSTEREO_SHARED)
    add_library(libstereo SHARED
                src/stereo_gradientstree.cu
                src/stereo_gtsgm.cu
                src/stereo_misgm.cu
                src/algorithms/censussgm.cu
                src/algorithms/excensussgm.cu
                src/algorithms/stablesgm.cu
                src/algorithms/tcensussgm.cu
                src/algorithms/hcensussgm.cu
                src/algorithms/hwcensussgm.cu
                src/algorithms/brefcensussgm.cu
                src/algorithms/meancensussgm.cu
                src/algorithms/hstablesgm.cu
                #src/stereo_hier_census.cu
                src/stereo_wcensussgm.cu
                src/stereo_census_adaptive.cu
                src/stereo_cp_censussgm.cu
                src/stereo_adcensussgm.cu
                src/stereo_varcensussgm.cu
                src/stereo_adsgm.cu
                src/stereo_sad.cu
                #src/stereo_sgmp.cpp
                src/stereo_wadcensus.cu
                src/costs/census.cu
                src/costs/tcensus.cu
                src/costs/gradient.cu
                src/costs/sad.cu
                src/costs/stable.cu
                src/costs/mutual_information.cu
                src/median_filter.cu
                src/median_filter_fixstars.cu
                src/dsi_tools.cu
                src/costs/gct.cu
                src/algorithms/gct.cu
    )
    set_target_properties(libstereo PROPERTIES PUBLIC_HEADER include/stereo.hpp)

else()
    add_library(libstereo
                src/stereo_gradientstree.cu
                src/stereo_gtsgm.cu
                src/stereo_misgm.cu
                src/algorithms/censussgm.cu
                src/algorithms/excensussgm.cu
                src/algorithms/stablesgm.cu
                src/algorithms/tcensussgm.cu
                src/algorithms/hcensussgm.cu
                src/algorithms/hwcensussgm.cu
                src/algorithms/brefcensussgm.cu
                src/algorithms/meancensussgm.cu
                src/algorithms/hstablesgm.cu
                #src/stereo_hier_census.cu
                src/stereo_wcensussgm.cu
                src/stereo_census_adaptive.cu
                src/stereo_cp_censussgm.cu
                src/stereo_adcensussgm.cu
                src/stereo_varcensussgm.cu
                src/stereo_adsgm.cu
                src/stereo_sad.cu
                #src/stereo_sgmp.cpp
                src/stereo_wadcensus.cu
                src/costs/census.cu
                src/costs/tcensus.cu
                src/costs/gradient.cu
                src/costs/sad.cu
                src/costs/stable.cu
                src/costs/mutual_information.cu
                src/median_filter.cu
                src/median_filter_fixstars.cu
                src/dsi_tools.cu
                src/costs/gct.cu
                src/algorithms/gct.cu
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
