# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})

set(REPSRC
	src/main.cpp
)

add_executable(ftl-reconstruct2 ${REPSRC})

#target_include_directories(ftl-reconstruct PUBLIC
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<INSTALL_INTERFACE:include>
#	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftl-reconstruct2 PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-reconstruct2 ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlctrl ftlnet ftlrender ftloperators ftlstreams ftlaudio)
