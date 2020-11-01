set(DISPSRC
	src/features.cu
)

add_library(ftldisparity ${DISPSRC})

target_include_directories(ftldisparity PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

target_link_libraries(ftldisparity ftlcommon cudatl Eigen3::Eigen Threads::Threads ${OpenCV_LIBS})

target_precompile_headers(ftldisparity REUSE_FROM ftlcommon)

set_property(TARGET ftldisparity PROPERTY CUDA_ARCHITECTURES OFF)

#if (BUILD_TESTS)
#	add_subdirectory(test)
#endif()
