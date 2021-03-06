add_library(voltu SHARED
	private/system.cpp
	private/feed_impl.cpp
	private/room_impl.cpp
	private/frame_impl.cpp
	private/image_impl.cpp
	private/observer_impl.cpp
	private/pointcloud_impl.cpp
	private/pipeline_impl.cpp
	private/operator_impl.cpp
	private/property_impl.cpp
)

file(READ "public/include/voltu/voltu.hpp" VOLVER)

string(REGEX MATCH "VOLTU_VERSION_MAJOR ([0-9]*)" _ ${VOLVER})
set(VOLTU_MAJOR ${CMAKE_MATCH_1})

string(REGEX MATCH "VOLTU_VERSION_MINOR ([0-9]*)" _ ${VOLVER})
set(VOLTU_MINOR ${CMAKE_MATCH_1})

string(REGEX MATCH "VOLTU_VERSION_PATCH ([0-9]*)" _ ${VOLVER})
set(VOLTU_PATCH ${CMAKE_MATCH_1})

message("VolTu SDK version: ${VOLTU_MAJOR}.${VOLTU_MINOR}.${VOLTU_PATCH}")

set_target_properties( voltu  PROPERTIES
	VERSION "${VOLTU_MAJOR}.${VOLTU_MINOR}"
	SOVERSION "${VOLTU_MAJOR}.${VOLTU_MINOR}"
)

target_include_directories(voltu
	PUBLIC public/include
	PRIVATE src)

target_link_libraries(voltu ftlcommon ftldata ftlctrl ftlrgbd ftlstreams ftlrender Threads::Threads ${OpenCV_LIBS} openvr ftlnet nanogui ${NANOGUI_EXTRA_LIBS} ceres nvidia-ml)

set(SDK_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk")

if (WIN32)
	install(TARGETS voltu RUNTIME COMPONENT runtime)
else()
	install(TARGETS voltu LIBRARY COMPONENT runtime)
endif()

set(voltu_sdk_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/lib/voltu_sdk_install)

ExternalProject_Add(
	voltu_sdk
	SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/public"
	BINARY_DIR ${SDK_BINARY_DIR}
	BUILD_ALWAYS true
	CMAKE_ARGS -DOpenCV_DIR=${OpenCV_DIR} -DWITH_PYTHON=True -DCMAKE_INSTALL_PREFIX=${voltu_sdk_DESTDIR}
)

install(DIRECTORY ${voltu_sdk_DESTDIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX})

if (BUILD_TESTS)
	add_subdirectory(tests)
endif()
