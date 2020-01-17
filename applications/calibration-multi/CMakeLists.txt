set(CALIBMULTI
	src/main.cpp
	src/visibility.cpp
	src/util.cpp
	src/multicalibrate.cpp
)

add_executable(ftl-calibrate-multi ${CALIBMULTI})

target_include_directories(ftl-calibrate-multi PRIVATE src)

target_link_libraries(ftl-calibrate-multi ftlcommon ftlnet ftlrgbd ftlstreams Threads::Threads ${OpenCV_LIBS} ${cvsba_LIBS})
