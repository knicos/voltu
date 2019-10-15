set(RGBDSRC
	src/sources/stereovideo/calibrate.cpp
	src/sources/stereovideo/local.cpp
	src/disparity.cpp
	src/source.cpp
	src/frame.cpp
	src/frameset.cpp
	src/sources/stereovideo/stereovideo.cpp
	src/sources/middlebury/middlebury_source.cpp
	src/sources/net/net.cpp
	src/streamer.cpp
	src/colour.cpp
	src/group.cpp
#	src/algorithms/rtcensus.cpp
#	src/algorithms/rtcensus_sgm.cpp
#	src/algorithms/opencv_sgbm.cpp
#	src/algorithms/opencv_bm.cpp
	src/cb_segmentation.cpp
	src/abr.cpp
	src/offilter.cpp
	src/sources/virtual/virtual.cpp
	src/sources/ftlfile/file_source.cpp
	src/sources/ftlfile/player.cpp
)

if (HAVE_REALSENSE)
	list(APPEND RGBDSRC "src/sources/realsense/realsense_source.cpp")
endif()

if (LibArchive_FOUND)
	list(APPEND RGBDSRC
		src/sources/snapshot/snapshot.cpp
		src/sources/snapshot/snapshot_source.cpp
	)
endif (LibArchive_FOUND)

if (LIBSGM_FOUND)
	list(APPEND RGBDSRC "src/algorithms/fixstars_sgm.cpp")
endif (LIBSGM_FOUND)

if (CUDA_FOUND)
	list(APPEND RGBDSRC
		src/algorithms/disp2depth.cu
		src/algorithms/offilter.cu
#		"src/algorithms/opencv_cuda_bm.cpp"
#		"src/algorithms/opencv_cuda_bp.cpp"
#		"src/algorithms/rtcensus.cu"
#		"src/algorithms/rtcensus_sgm.cu"
#		"src/algorithms/consistency.cu"
#		"src/algorithms/sparse_census.cu"
#		"src/algorithms/tex_filter.cu"
#		"src/algorithms/nick1.cu"
#		"src/algorithms/nick.cpp")
)
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
target_link_libraries(ftlrgbd ftlcommon ${OpenCV_LIBS} ${LIBSGM_LIBRARIES} ${CUDA_LIBRARIES} Eigen3::Eigen ${REALSENSE_LIBRARY} ftlnet ${LibArchive_LIBRARIES} ftlcodecs)

add_subdirectory(test)

