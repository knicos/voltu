set(RGBDSRC
	src/calibrate.cpp
	src/local.cpp
	src/disparity.cpp
	src/rgbd_source.cpp
	src/stereovideo_source.cpp
	src/net_source.cpp
	src/algorithms/rtcensus.cpp
	src/algorithms/rtcensus_sgm.cpp
	src/algorithms/opencv_sgbm.cpp
	src/algorithms/opencv_bm.cpp
)

if (LIBSGM_FOUND)
	list(APPEND RGBDSRC "src/algorithms/fixstars_sgm.cpp")
endif (LIBSGM_FOUND)

if (CUDA_FOUND)
	list(APPEND RGBDSRC
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

add_library(ftlrgbd ${RGBDSRC})

# target_compile_options(ftlrgbd PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
# target_compile_options(ftlrgbd PUBLIC "-DMAKE_SHARED")

target_include_directories(ftlrgbd PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftlrgbd PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlrgbd ftlcommon Threads::Threads ${OpenCV_LIBS} ${LIBSGM_LIBRARIES} ${CUDA_LIBRARIES} Eigen3::Eigen glog::glog ftlnet)

