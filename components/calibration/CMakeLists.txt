set(CALIBSRC
	src/parameters.cpp
	src/extrinsic.cpp
	src/structures.cpp
	src/visibility.cpp
	src/object.cpp
)

if (WITH_CERES)
	list(APPEND CALIBSRC src/optimize.cpp)
	set_source_files_properties(src/optimize.cpp PROPERTIES COMPILE_FLAGS -O3)
endif()

add_library(ftlcalibration ${CALIBSRC})

target_include_directories(ftlcalibration
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE
	src/
	${OpenCV_INCLUDE_DIRS}
)

# ftlcodecs required for ftl::data::Channel
target_link_libraries(ftlcalibration ftlcommon ftlcodecs Threads::Threads ${OpenCV_LIBS} Eigen3::Eigen ceres)

if (BUILD_TESTS)
	ADD_SUBDIRECTORY(test)
endif()
