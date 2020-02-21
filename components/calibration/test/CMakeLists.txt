### Calibration ################################################################
add_executable(calibration_parameters_unit
	./tests.cpp
	./test_parameters.cpp
	../src/parameters.cpp
)

target_include_directories(calibration_parameters_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(calibration_parameters_unit ftlcommon Threads::Threads ${OS_LIBS} ${OpenCV_LIBS})
add_test(CalibrationValidateTest calibration_parameters_unit)