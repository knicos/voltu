set(FTLMERGER
	src/main.cpp
)

add_executable(ftl-merge ${FTLMERGER})

target_include_directories(ftl-merge PRIVATE src)

target_link_libraries(ftl-merge ftlcommon ftlcodecs ftlrgbd Threads::Threads ${OpenCV_LIBS})


