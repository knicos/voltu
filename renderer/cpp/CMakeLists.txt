# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/net/cpp/include)
#include_directories(${PROJECT_BINARY_DIR})


add_library(ftlrender
	src/display.cpp
)

# target_include_directories(ftl-vision PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(ftlrender PUBLIC ${PCL_DEFINITIONS})

target_include_directories(ftlrender PUBLIC
${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlrender Threads::Threads glog::glog ${OpenCV_LIBS} ${PCL_LIBRARIES})



#ADD_SUBDIRECTORY(test)
