
#add_library(Catch INTERFACE)
#target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

add_executable(visibility_unit
	./tests.cpp
	../src/visibility.cpp
	./visibility_unit.cpp
)

target_include_directories(visibility_unit PUBLIC ../src/)
target_link_libraries(visibility_unit pthread dl ftlcommon)

add_test(VisibilityTest visibility_unit)

################################################################################

add_executable(calibration_unit
	./tests.cpp
	../src/calibration.cpp
	../src/optimization.cpp
	../src/calibration_data.cpp
	./calibration_unit.cpp
)

target_include_directories(calibration_unit PUBLIC ../src/)
target_link_libraries(calibration_unit pthread dl ftlcommon Eigen3::Eigen ceres)

add_test(CalibrationTest calibration_unit WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
