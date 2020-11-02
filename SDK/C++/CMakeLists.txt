add_library(voltu SHARED
	private/system.cpp
	private/feed_impl.cpp
	private/room_impl.cpp
	private/frame_impl.cpp
	private/image_impl.cpp
	private/observer_impl.cpp
	private/pointcloud_impl.cpp
)

target_include_directories(voltu
	PUBLIC public/include
	PRIVATE src)

target_link_libraries(voltu ftlcommon ftldata ftlctrl ftlrgbd ftlstreams ftlrender Threads::Threads ${OpenCV_LIBS} openvr ftlnet nanogui ${NANOGUI_EXTRA_LIBS} ceres nvidia-ml)

ExternalProject_Add(
	voltu_sdk
	SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/public"
	BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk"
	INSTALL_COMMAND ""
	BUILD_ALWAYS true
	CMAKE_ARGS -DOpenCV_DIR=${OpenCV_DIR}
)
