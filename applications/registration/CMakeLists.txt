set(REGSRC
	src/main.cpp
	src/manual.cpp
	src/correspondances.cpp
	src/sfm.cpp
	src/aruco.cpp
	src/common.cpp
)

add_executable(ftl-register ${REGSRC})

target_include_directories(ftl-register PRIVATE src)

target_link_libraries(ftl-register ftlcommon ftlnet ftlrgbd Threads::Threads ${OpenCV_LIBS})

