set(CALIBMULTI
	src/main.cpp
	src/visibility.cpp
	src/util.cpp
	src/multicalibrate.cpp
	../calibration-ceres/src/calibration.cpp
)

add_executable(ftl-calibrate-multi ${CALIBMULTI})

target_include_directories(ftl-calibrate-multi PRIVATE src ../calibration-ceres/src)

target_link_libraries(ftl-calibrate-multi ftlcalibration ftlcommon ftlnet ftlrgbd ftlstreams Threads::Threads ${OpenCV_LIBS} ceres)
