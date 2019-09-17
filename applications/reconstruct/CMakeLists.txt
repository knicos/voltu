# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})

set(REPSRC
	src/main.cpp
	src/voxel_scene.cpp
	src/scene_rep_hash_sdf.cu
	src/compactors.cu
	src/garbage.cu
	src/integrators.cu
	#src/ray_cast_sdf.cu
	src/voxel_render.cu
	src/camera_util.cu
	src/voxel_hash.cu
	src/voxel_hash.cpp
	#src/ray_cast_sdf.cpp
	src/registration.cpp
	#src/virtual_source.cpp
	src/splat_render.cpp
	src/dibr.cu
	src/mls.cu
	src/depth_camera.cu
	src/depth_camera.cpp
)

add_executable(ftl-reconstruct ${REPSRC})
add_dependencies(ftl-reconstruct ftlnet)
add_dependencies(ftl-reconstruct ftlcommon)

target_include_directories(ftl-reconstruct PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET ftl-reconstruct PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-reconstruct ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlctrl ftlnet)


