add_executable (ftl-calibration-ceres
                src/main.cpp
                src/calibration_data.cpp
                src/visibility.cpp
                src/calibration.cpp
)

target_include_directories(ftl-calibration-ceres PRIVATE src/)
target_include_directories(ftl-calibration-ceres PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ftl-calibration-ceres ftlcalibration Threads::Threads ftlcommon Eigen3::Eigen ceres)

add_subdirectory(test)
