set(FTL2MKVSRC
	src/main.cpp
)

add_executable(ftl2mkv ${FTL2MKVSRC})

target_include_directories(ftl2mkv PRIVATE src)

target_link_libraries(ftl2mkv ftlcommon ftlcodecs ftlrgbd Threads::Threads ${OpenCV_LIBS} avutil avformat avcodec)


