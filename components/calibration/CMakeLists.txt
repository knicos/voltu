set(CALIBSRC
	src/parameters.cpp
)

if (WITH_CERES)
	list(APPEND CALIBSRC src/optimize.cpp)
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

target_link_libraries(ftlcalibration ftlcommon Threads::Threads ${OpenCV_LIBS} Eigen3::Eigen ceres)

if (BUILD_TESTS)
	ADD_SUBDIRECTORY(test)
endif()
