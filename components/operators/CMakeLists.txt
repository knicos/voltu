add_library(ftloperators
	src/smoothing.cpp
	src/smoothing.cu
	src/mls.cu
	src/smoothchan.cu
	src/operator.cpp
	src/colours.cpp
	src/normals.cpp
	src/filling.cpp
	src/filling.cu
	src/nvopticalflow.cpp
)

# These cause errors in CI build and are being removed from PCL in newer versions
# target_compile_options(ftlrender PUBLIC ${PCL_DEFINITIONS})

target_include_directories(ftloperators PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftloperators ftlrender ftlrgbd ftlcommon Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

#ADD_SUBDIRECTORY(test)
