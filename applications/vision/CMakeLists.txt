# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/vision/include)
#include_directories(${PROJECT_BINARY_DIR})

#add_subdirectory(lib)

set(CVNODESRC
	src/main.cpp
	#src/middlebury.cpp
)

add_executable(voltu-vision ${CVNODESRC})
install(TARGETS voltu-vision RUNTIME COMPONENT vision)

target_include_directories(voltu-vision PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET voltu-vision PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(voltu-vision ftlrgbd ftlcommon ftlstreams ftlctrl ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlnet ftlaudio)

target_precompile_headers(voltu-vision REUSE_FROM ftldata)
