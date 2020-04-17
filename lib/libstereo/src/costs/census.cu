#include "census.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cuda_runtime.h>

namespace algorithms {
	template<int WINX, int WINY>
	struct CensusTransform {
		static_assert(WINX*WINY <= 64, "Census window is too large");

		__host__ __device__ inline uint64_t ct_window(const int y, const int x) {
			uint64_t ct = 0;
			uchar center = im(y, x);

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const int y_ = y + wy;
					const int x_ = x + wx;
					ct = ct << 1;
					ct |= (center < (im(y_,x_)) ? 1 : 0);
				}
			}
			return ct;
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			//for (int y = WINY/2+1; y < size.y - (WINY/2+1); y++) {
			//	for (int x = WINX/2+1; x < size.x - (WINX/2+1); x++) {
			for (int y = thread.y+WINY/2; y<size.y-WINY/2-1; y+=stride.y) {
				for (int x = thread.x+WINX/2; x<size.x-WINX/2-1; x+=stride.x) {
					out(y, x) = ct_window(y, x);
				}
			}
		}

		Array2D<uchar>::Data im;
		Array2D<uint64_t>::Data out;
	};
}

void CensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	parallel2D<algorithms::CensusTransform<9,7>>({l.data(), ct_l_.data()}, l.width, l.height);
	parallel2D<algorithms::CensusTransform<9,7>>({r.data(), ct_r_.data()}, r.width, r.height);
}

void CensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
	if (l.type() != CV_8UC1 || r.type() != CV_8UC1) { throw std::exception(); }
	if (l.rows() != r.rows() || l.cols() != r.cols() || l.rows() != height() || l.cols() != width()) {
		throw std::exception();
	}

	if (l.isGpuMat() && r.isGpuMat()) {
		auto ml = l.getGpuMat();
		auto mr = r.getGpuMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else if (l.isMat() && r.isMat()) {
		auto ml = l.getMat();
		auto mr = r.getMat();
		set(Array2D<uchar>(ml), Array2D<uchar>(mr));
	}
	else {
		throw std::exception();
	}
}
