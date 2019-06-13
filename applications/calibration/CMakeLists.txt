set(CALIBSRC
	src/main.cpp
	src/lens.cpp
	src/stereo.cpp
	src/align.cpp
	src/common.cpp
)

add_executable(ftl-calibrate ${CALIBSRC})

target_include_directories(ftl-calibrate PRIVATE src)

target_link_libraries(ftl-calibrate ftlcommon Threads::Threads ${OpenCV_LIBS})


