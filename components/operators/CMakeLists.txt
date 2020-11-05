set(OPERSRC
	src/surface/smoothing.cpp
	src/surface/smoothing.cu
	src/surface/mls/image_basic.cu
	src/surface/mls.cu
	src/analysis/local/smoothchan.cu
	src/operator.cpp
	src/misc/colours.cpp
	src/analysis/local/normals.cpp
	src/surface/filling.cpp
	src/surface/filling.cu
	src/disparity/libstereo.cpp
	src/disparity/disp2depth.cu
	src/disparity/disparity_to_depth.cpp
	src/disparity/bilateral_filter.cpp
	src/disparity/opencv/disparity_bilateral_filter.cpp
	src/disparity/opencv/disparity_bilateral_filter.cu
	src/analysis/segmentation/segmentation.cu
	src/analysis/segmentation/segmentation.cpp
	src/analysis/local/mask.cu
	src/analysis/local/mask.cpp
	src/misc/antialiasing.cpp
	src/misc/antialiasing.cu
	src/fusion/mvmls.cpp
	src/fusion/correspondence.cu
	src/fusion/correspondence_depth.cu
	src/fusion/correspondence_util.cu
	src/fusion/mls_aggr.cu
	src/fusion/smoothing/mls_multi_weighted.cu
	src/fusion/carving/carver.cu
	src/fusion/fusion.cpp
	src/misc/clipping.cpp
	src/disparity/depth.cpp
	src/analysis/tracking/detectandtrack.cpp
	src/analysis/tracking/aruco.cpp
	src/analysis/local/weighting.cpp
	src/analysis/local/weighting.cu
	src/misc/poser.cpp
	src/analysis/evaluation/gt_analysis.cpp
	src/analysis/evaluation/gt_analysis.cu
)

if (HAVE_LIBSGM)
	list(APPEND OPERSRC src/disparity/fixstars_sgm.cpp)
endif (HAVE_LIBSGM)

if (HAVE_OPTFLOW)
	list(APPEND OPERSRC
		src/analysis/tracking/nvopticalflow.cpp
		src/analysis/tracking/opticalflow.cu
		src/disparity/optflow_smoothing.cu
		src/disparity/optflow_smoothing.cpp)
endif()

add_library(ftloperators ${OPERSRC})

target_include_directories(ftloperators PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

target_link_libraries(ftloperators ftlrender ftlrgbd ftlcommon sgm libstereo Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

target_precompile_headers(ftloperators REUSE_FROM ftldata)

set_property(TARGET ftloperators PROPERTY CUDA_ARCHITECTURES OFF)

if (BUILD_TESTS)
	add_subdirectory(test)
endif()
