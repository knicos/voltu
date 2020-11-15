set(SDKSRC
	src/common.cpp
	src/streams.cpp
)

add_library(ftl-dev SHARED ${SDKSRC})
set_target_properties(ftl-dev PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(ftl-dev PROPERTIES SOVERSION 0)


target_include_directories(ftl-dev PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftl-dev PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

target_link_libraries(ftl-dev ftlcommon ftlcodecs ftlrgbd ftlstreams Threads::Threads ${OpenCV_LIBS} ftlnet)

#install(TARGETS ftl-dev
#    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
	
add_subdirectory(examples/image_write)
add_subdirectory(examples/video_write)
