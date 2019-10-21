set(PLAYERSRC
	src/main.cpp
)

add_executable(ftl-player ${PLAYERSRC})

target_include_directories(ftl-player PRIVATE src)

target_link_libraries(ftl-player ftlcommon ftlcodecs ftlrgbd Threads::Threads ${OpenCV_LIBS})


