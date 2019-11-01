add_library(ftlfilter
    src/smoothing.cpp
    src/smoothing.cu
)

# These cause errors in CI build and are being removed from PCL in newer versions
# target_compile_options(ftlrender PUBLIC ${PCL_DEFINITIONS})

target_include_directories(ftlfilter PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlfilter ftlrender ftlrgbd ftlcommon Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

#ADD_SUBDIRECTORY(test)
