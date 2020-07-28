### Calibration ################################################################
add_executable(calibration_parameters_unit
	./tests.cpp
	./test_parameters.cpp
	../src/parameters.cpp
)

target_include_directories(calibration_parameters_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(calibration_parameters_unit ftlcalibration ftlcommon ftlcodecs Threads::Threads ${OS_LIBS} ${OpenCV_LIBS})
add_test(CalibrationValidateTest calibration_parameters_unit)

### Calibration Helper #########################################################
add_executable(calibration_helper_unit
	./tests.cpp
	./test_helper.cpp
	../src/extrinsic.cpp
)

target_include_directories(calibration_helper_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(calibration_helper_unit ftlcalibration ftlcommon Threads::Threads ${OS_LIBS} ${OpenCV_LIBS})
add_test(CalibrationHelperTest calibration_helper_unit)

### Extrinsic calib ############################################################

add_executable(calibration_extrinsic_unit
	./tests.cpp
	./test_extrinsic.cpp
	../src/extrinsic.cpp
)

target_include_directories(calibration_extrinsic_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(calibration_extrinsic_unit ftlcalibration ftlcommon Threads::Threads ${OS_LIBS} ${OpenCV_LIBS})
add_test(CalibrationExtrinsicTest calibration_extrinsic_unit)
