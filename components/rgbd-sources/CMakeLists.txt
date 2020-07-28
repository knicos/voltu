set(RGBDSRC
	src/sources/stereovideo/rectification.cpp
	src/sources/stereovideo/opencv.cpp
	src/source.cpp
	src/frame.cpp
	#src/frameset.cpp
	src/sources/stereovideo/stereovideo.cpp
	#src/colour.cpp
	#src/group.cpp
	#src/cb_segmentation.cpp
	#src/abr.cpp
	src/sources/screencapture/screencapture.cpp
	src/camera.cpp
	#src/init.cpp
)

if (HAVE_REALSENSE)
	list(APPEND RGBDSRC "src/sources/realsense/realsense_source.cpp")
endif()

if (HAVE_PYLON)
	list(APPEND RGBDSRC "src/sources/stereovideo/pylon.cpp")
endif()

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

target_link_libraries(ftlrgbd ftlcalibration ftlcommon ${OpenCV_LIBS} ${CUDA_LIBRARIES} Eigen3::Eigen realsense ftlnet ${LibArchive_LIBRARIES} ftlcodecs ftloperators ftldata ${X11_X11_LIB} ${X11_Xext_LIB} ${X11_Xtst_LIB} ${X11_XTest_LIB} Pylon)

target_precompile_headers(ftlrgbd REUSE_FROM ftldata)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

