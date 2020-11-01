
add_library(cudatl INTERFACE)

target_include_directories(cudatl INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include)

#target_link_libraries(ftldata ftlcommon Eigen3::Eigen ftlcodecs)

#set_property(TARGET ftldata PROPERTY CUDA_ARCHITECTURES OFF)

#if (BUILD_TESTS)
#add_subdirectory(test)
#endif()

