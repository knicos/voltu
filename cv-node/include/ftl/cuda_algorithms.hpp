#ifndef _FTL_CUDA_ALGORITHMS_HPP_
#define _FTL_CUDA_ALGORITHMS_HPP_

#include <ftl/cuda_common.hpp>

namespace ftl {
namespace cuda {

	void consistency(const TextureObject<float> &dl, const TextureObject<float> &dr,
			TextureObject<float> &disp);
			
	void sparse_census(const TextureObject<uchar4> &l, const TextureObject<uchar4> &r,
			TextureObject<uint2> &cl, TextureObject<uint2> &cr);

	void texture_filter(const TextureObject<uchar4> &t, const TextureObject<float> &d,
			TextureObject<float> &f, int num_disp, double thresh);
			
	void texture_map(const TextureObject<uchar4> &t,
			TextureObject<float> &f);

}
}

#endif // _FTL_CUDA_ALGORITHMS_HPP_

