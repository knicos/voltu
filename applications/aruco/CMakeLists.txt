set(FTLARUCOSRC
	src/main.cpp
)

add_executable(ftl-aruco ${FTLARUCOSRC})

target_include_directories(ftl-aruco PRIVATE src)

target_link_libraries(ftl-aruco ftlcommon Threads::Threads ${OpenCV_LIBS})


