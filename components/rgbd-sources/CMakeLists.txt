set(RGBDSRC
	src/sources/stereovideo/calibrate.cpp
	src/sources/stereovideo/local.cpp
	src/source.cpp
	src/frame.cpp
	src/frameset.cpp
	src/sources/stereovideo/stereovideo.cpp
	#src/sources/net/net.cpp
	#src/streamer.cpp
	#src/colour.cpp
	src/group.cpp
	src/cb_segmentation.cpp
	#src/abr.cpp
	#src/sources/virtual/virtual.cpp
	#src/sources/ftlfile/file_source.cpp
	#src/sources/ftlfile/player.cpp
	src/sources/screencapture/screencapture.cpp
	src/camera.cpp
)

if (HAVE_REALSENSE)
	list(APPEND RGBDSRC "src/sources/realsense/realsense_source.cpp")
endif()

if (LibArchive_FOUND)
	list(APPEND RGBDSRC
		src/sources/snapshot/snapshot.cpp
		src/sources/snapshot/snapshot_source.cpp
	)
endif (LibArchive_FOUND)

add_library(ftlrgbd ${RGBDSRC})

# target_compile_options(ftlrgbd PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
# target_compile_options(ftlrgbd PUBLIC "-DMAKE_SHARED")

target_include_directories(ftlrgbd PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftlrgbd PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

target_link_libraries(ftlrgbd ftlcommon ${OpenCV_LIBS} ${LIBSGM_LIBRARIES} ${CUDA_LIBRARIES} Eigen3::Eigen realsense ftlnet ${LibArchive_LIBRARIES} ftlcodecs ftloperators ftldata ${X11_X11_LIB} ${X11_Xext_LIB})

if (BUILD_TESTS)
add_subdirectory(test)
endif()

