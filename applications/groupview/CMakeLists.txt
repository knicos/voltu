set(GVIEWSRC
	src/main.cpp
)

add_executable(ftl-view ${GVIEWSRC})

target_include_directories(ftl-view PRIVATE src)

target_link_libraries(ftl-view ftlcommon ftlnet ftlrgbd Threads::Threads ${OpenCV_LIBS})


