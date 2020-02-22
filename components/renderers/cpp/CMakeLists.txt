add_library(ftlrender
	src/normals.cu
	src/mask.cu
	src/screen.cu
	src/clipping.cu
	src/dibr.cu
	src/triangle_render.cu
	src/reprojection.cu
	src/CUDARender.cpp
	src/colouriser.cpp
	src/colour_util.cu
)

target_include_directories(ftlrender PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlrender ftlrgbd ftlcommon Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

#ADD_SUBDIRECTORY(test)
