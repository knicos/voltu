set(RECSRC
    src/main.cpp
    src/registration.cpp
)

add_executable(ftl-record ${RECSRC})

target_include_directories(ftl-record PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-record ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlctrl ftlnet)


