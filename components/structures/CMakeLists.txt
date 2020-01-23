
add_library(ftldata INTERFACE)

target_include_directories(ftldata INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ftldata INTERFACE ftlcommon Eigen3::Eigen ftlcodecs)

#add_subdirectory(test)

