# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})

set(GUISRC
	src/main.cpp
	src/ctrl_window.cpp
	src/src_window.cpp
	src/config_window.cpp
	src/pose_window.cpp
	src/screen.cpp
	src/gltexture.cpp
	src/camera.cpp
	src/media_panel.cpp
	src/thumbview.cpp
)

add_executable(ftl-gui ${GUISRC})

target_include_directories(ftl-gui PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#if (CUDA_FOUND)
#set_property(TARGET ftl-gui PROPERTY CUDA_SEPARABLE_COMPILATION ON)
#endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-gui ftlcommon ftlctrl ftlrgbd Threads::Threads ${OpenCV_LIBS} glog::glog ftlnet ftlrender nanogui GL)


