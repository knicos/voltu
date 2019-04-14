# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/vision/include)
#include_directories(${PROJECT_BINARY_DIR})

add_subdirectory(lib)

set(CVNODESRC
	src/main.cpp
	src/calibrate.cpp
	src/local.cpp
	src/sync.cpp
	src/disparity.cpp
	src/streamer.cpp
	src/middlebury.cpp
	src/algorithms/rtcensus.cpp
	src/algorithms/rtcensus_sgm.cpp
	src/algorithms/opencv_sgbm.cpp
	src/algorithms/opencv_bm.cpp
	src/algorithms/elas.cpp
)

if (LIBSGM_FOUND)
	list(APPEND CVNODESRC "src/algorithms/fixstars_sgm.cpp")
endif (LIBSGM_FOUND)

if (CUDA_FOUND)
	list(APPEND CVNODESRC
		"src/algorithms/opencv_cuda_bm.cpp"
		"src/algorithms/opencv_cuda_bp.cpp"
		"src/algorithms/rtcensus.cu"
		"src/algorithms/rtcensus_sgm.cu"
		"src/algorithms/consistency.cu"
		"src/algorithms/sparse_census.cu"
		"src/algorithms/tex_filter.cu"
		"src/algorithms/nick1.cu"
		"src/algorithms/nick.cpp")
endif (CUDA_FOUND)

add_executable(ftl-vision ${CVNODESRC})
add_dependencies(ftl-vision ftlcommon)
add_dependencies(ftl-vision ftlnet)
add_dependencies(ftl-vision ftlrender)
add_dependencies(ftl-vision libelas)

if (CUDA_FOUND)
set_property(TARGET ftl-vision PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-vision ftlcommon ftlrender Threads::Threads ZLIB::ZLIB libelas ${OpenCV_LIBS} ${LIBSGM_LIBRARIES} ${CUDA_LIBRARIES} glog::glog ftlnet)


