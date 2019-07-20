#ifndef _FTL_RECONSTRUCTION_SPLAT_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_HPP_

#include <ftl/configurable.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/voxel_scene.hpp>
//#include <ftl/ray_cast_util.hpp>
#include <ftl/cuda_common.hpp>

#include "splat_params.hpp"

namespace ftl {
namespace render {

/**
 * Render the voxel hash structure by generating image points for surface
 * voxels and expanding those into interpolated splats. This is a two pass
 * algorithm with the option of completing the second pass on a separate GPU.
 * It also possible to only complete the first pass and perform the second step
 * on a separate machine or at a later time, the advantage being to save local
 * processing resources and that the first pass result may compress better.
 */
class Splatter {
	public:
	explicit Splatter(ftl::voxhash::SceneRep *scene);
	~Splatter();

	void render(ftl::rgbd::Source *src, cudaStream_t stream=0);

	void setOutputDevice(int);

	private:
	int device_;
	ftl::cuda::TextureObject<uint> depth1_;
	ftl::cuda::TextureObject<uchar4> colour1_;
	ftl::cuda::TextureObject<float> depth2_;
	ftl::cuda::TextureObject<uchar4> colour2_;
	SplatParams params_;
	ftl::voxhash::SceneRep *scene_;
};

}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_HPP_
