# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/vision/include)
#include_directories(${PROJECT_BINARY_DIR})

#add_subdirectory(lib)

set(CVNODESRC
	src/main.cpp
	src/streamer.cpp
	src/middlebury.cpp
)

add_executable(ftl-vision ${CVNODESRC})

target_include_directories(ftl-vision PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftl-vision PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-vision ftlrgbd ftlcommon ftlrender Threads::Threads ${OpenCV_LIBS} ${LIBSGM_LIBRARIES} ${CUDA_LIBRARIES} glog::glog ftlnet)


