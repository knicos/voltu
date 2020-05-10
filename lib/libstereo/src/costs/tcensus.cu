#include "tcensus.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cuda_runtime.h>

namespace algorithms {
	template<int WINX, int WINY>
	struct TCensusTransform {

		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out) {

			static_assert(BPP == 2, "2 bits per pixel expected");
			short center = im(y, x);
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const int y_ = y + wy;
					const int x_ = x + wx;

					// If fist value, set zero. Otherwise shift left by BPP
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << BPP); }

					if 		(center+t > im(y_,x_))	{ *out |= 1;	/* 01 */ }
					else if (center-t < im(y_,x_))	{ *out |= 2;	/* 10 */ }
					else 							{				/* 00 */ }

					i += BPP;
					// if all bits set, continue to next element
					if (i % 64 == 0) { out++; }
				}
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y+WINY/2; y<size.y-WINY/2-1; y+=stride.y) {
				for (int x = thread.x+WINX/2; x<size.x-WINX/2-1; x+=stride.x) {
					window(y, x, &(out(y, x*WSTEP)));
				}
			}
		}

		Array2D<uchar>::Data im;
		Array2D<uint64_t>::Data out;
		short t; // intensity threshold

		// Bits per pixel (for each census feature). Must be 1 or power of 2 for
		// window() to be correct (for tri-census)!
		static constexpr int BPP = 2;
		// number of uint64_t values for each window
		static constexpr int WSTEP = (BPP*WINX*WINY-1)/(sizeof(uint64_t)*8) + 1;
	};
}

void TCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	parallel2D<algorithms::TCensusTransform<9,7>>({l.data(), ct_l_.data(), t_}, l.width, l.height);
	parallel2D<algorithms::TCensusTransform<9,7>>({r.data(), ct_r_.data(), t_}, r.width, r.height);
}

void TCensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
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
