#ifndef _FTL_LIBSTEREO_COSTS_GT_HPP_
#define _FTL_LIBSTEREO_COSTS_GT_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"
#include <cuda_runtime.h>

namespace impl {
	struct GtMatchingCost : DSImplBase<float> {
		typedef float Type;

		GtMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<float>({w,h,dmin,dmax}) {}

		__host__ __device__ inline float operator()(const int y, const int x, const int d) const {
			if (round(gt(y,x)) == float(d)) { return 0.0f; }
			else { return 1.0f; }
		}

		static constexpr float COST_MAX = 1.0f;

		Array2D<float>::Data gt;
	};
}

class GtMatchingCost : public DSBase<impl::GtMatchingCost> {
public:
	typedef impl::GtMatchingCost DataType;
	typedef float Type;

	GtMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	GtMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {}

	void set(cv::InputArray gt) {
		#if USE_GPU
		if (gt.isGpuMat()) {
			auto m = gt.getGpuMat();
			gt_ = Array2D<float>(m);
		}
		else if (gt.isMat()) {
			gt_.create(gt.cols(), gt.rows());
			gt_.toGpuMat().upload(gt.getMat());
		}
		#else
		if (gt.isGpuMat()) {
			gt_.create(gt.cols(), gt.rows());
			gt.getGpuMat().download(gt_.getMat());
		}
		else if (gt.isMat()) {
			auto m = gt.getMat();
			gt_ = Array2D<float>(m);
		}
		#endif

		data().gt = gt_.data();
	}

	void set(const Array2D<float>& gt) {
		gt_ = gt;
		data().gt = gt_.data();
	}

protected:
	Array2D<float> gt_;
};

#endif
