
add_library(ftldata ./src/new_frame.cpp ./src/pool.cpp)

target_include_directories(ftldata PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ftldata ftlcommon Eigen3::Eigen ftlcodecs)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

