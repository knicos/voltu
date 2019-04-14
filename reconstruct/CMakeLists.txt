# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})


set(REPSRC
	src/main.cpp
)

add_executable(ftl-reconstruct ${REPSRC})
add_dependencies(ftl-reconstruct ftlnet)
add_dependencies(ftl-reconstruct ftlcommon)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-reconstruct ftlcommon Threads::Threads ZLIB::ZLIB ${OpenCV_LIBS} glog::glog ftlnet ftlrender)


