# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})

set(SIMPVIEWSRC
	main.cpp
)

add_executable(simple-viewer ${SIMPVIEWSRC})

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(simple-viewer ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlctrl ftlnet ftlrender ftloperators ftlstreams ftlaudio)
