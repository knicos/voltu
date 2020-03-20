set(OPERSRC
	src/smoothing.cpp
	src/smoothing.cu
	src/mls.cu
	src/smoothchan.cu
	src/operator.cpp
	src/colours.cpp
	src/normals.cpp
	src/filling.cpp
	src/filling.cu
	src/disparity/disp2depth.cu
	src/disparity/disparity_to_depth.cpp
	src/disparity/bilateral_filter.cpp
	src/disparity/opencv/disparity_bilateral_filter.cpp
	src/disparity/opencv/disparity_bilateral_filter.cu
	src/segmentation.cu
	src/segmentation.cpp
	src/mask.cu
	src/mask.cpp
	src/antialiasing.cpp
	src/antialiasing.cu
	src/fusion/mvmls.cpp
	src/fusion/correspondence.cu
	src/fusion/correspondence_depth.cu
	src/fusion/correspondence_util.cu
	src/fusion/mls_aggr.cu
	src/clipping.cpp
	src/depth.cpp
	src/detectandtrack.cpp
	src/aruco.cpp
	src/weighting.cpp
	src/weighting.cu
	src/poser.cpp
	src/gt_analysis.cpp
	src/gt_analysis.cu
)

if (LIBSGM_FOUND)
	list(APPEND OPERSRC src/disparity/fixstars_sgm.cpp)
endif (LIBSGM_FOUND)

if (HAVE_OPTFLOW)
	list(APPEND OPERSRC
		src/nvopticalflow.cpp
		src/disparity/optflow_smoothing.cu
		src/disparity/optflow_smoothing.cpp)
endif()

add_library(ftloperators ${OPERSRC})

target_include_directories(ftloperators PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

target_link_libraries(ftloperators ftlrender ftlrgbd ftlcommon sgm Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

#ADD_SUBDIRECTORY(test)
