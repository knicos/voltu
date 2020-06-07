
add_library(ftldata STATIC ./src/new_frame.cpp)

target_include_directories(ftldata PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ftldata ftlcommon Eigen3::Eigen ftlcodecs)

add_subdirectory(test)

