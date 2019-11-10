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
	src/disparity/fixstars_sgm.cpp
	src/disparity/bilateral_filter.cpp
	src/segmentation.cu
	src/segmentation.cpp
)

if (HAVE_OPTFLOW)
	list(APPEND OPERSRC src/nvopticalflow.cpp
		src/disparity/optflow_smoothing.cu
		src/disparity/optflow_smoothing.cpp)
endif()

add_library(ftloperators ${OPERSRC})

# These cause errors in CI build and are being removed from PCL in newer versions
# target_compile_options(ftlrender PUBLIC ${PCL_DEFINITIONS})

target_include_directories(ftloperators PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftloperators ftlrender ftlrgbd ftlcommon Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

#ADD_SUBDIRECTORY(test)
