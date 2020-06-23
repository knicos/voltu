#include "census.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cuda_runtime.h>

template <typename T>
__host__ __device__ inline T square(T v) { return v*v; }

namespace algorithms {
	/** Census transform. Bits stored in row major order. */
	template<int WINX, int WINY>
	struct CensusTransformRowMajor {
		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out) {
			short center = im(y, x);
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const int y_ = y + wy;
					const int x_ = x + wx;

					// zero if first value, otherwise shift to left
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << 1); }
					*out |= (center < (im(y_,x_)) ? 1 : 0);

					i += 1;
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

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};

	/** Census transform. Bits stored in row major order. This variant uses a
	 *  nested window that expands with distance from center, where the
	 *  absolute max difference to center is used from that window. */
	template<int WINX, int WINY>
	struct ExCensusTransformRowMajor {
		__host__ __device__ inline short subwindow(const int x, const int y, int wx, int wy, short center, ushort2 size) {
			short value = center;

			int nwx = 3*wx; //int(float(wx)*scale);
			int nwy = 3*wy; //int(float(wy)*scale);
			//int wsize = max(0,max(abs(wx),abs(wy))-2);
			//if (x == 100 && y == 100) printf("Ex shape: %d, %d\n",nwx,nwy);
			for (int dy=-max(0,abs(wy)-2); dy<=max(0,abs(wy)-2); ++dy) {
			for (int dx=-max(0,abs(wx)-2); dx<=max(0,abs(wx)-2); ++dx) {
				auto v = im(min(size.y, max(0,y+nwy+dy)), min(size.x, max(0,x+nwx+dx)));
				if (abs(center-v) > abs(center-value)) value = v;
			}
			}

			return value;
		}

		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out, ushort2 size) {
			short center = im(y, x);
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const int y_ = y + wy;
					const int x_ = x + wx;

					// zero if first value, otherwise shift to left
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << 1); }
					*out |= (center < (subwindow(x,y,wx,wy,center,size)) ? 1 : 0);

					i += 1;
					// if all bits set, continue to next element
					if (i % 64 == 0) { out++; }
				}
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y+WINY/2; y<size.y-WINY/2-1; y+=stride.y) {
				for (int x = thread.x+WINX/2; x<size.x-WINX/2-1; x+=stride.x) {
					window(y, x, &(out(y, x*WSTEP)), size);
				}
			}
		}

		Array2D<uchar>::Data im;
		Array2D<uint64_t>::Data out;
		float scale;

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};

	/* W. S. Fife and J. K. Archibald,
	   "Improved Census Transforms for Resource-Optimized Stereo Vision,"
	  in IEEE Transactions on Circuits and Systems for Video Technology,
	  vol. 23, no. 1, pp. 60-73, Jan. 2013. */
	template<int WINX, int WINY>
	struct GCensusTransformRowMajor {
		__host__ __device__ inline void window(const int y, const int x, uint64_t* __restrict__ out) {
			//short center = im(y, x);
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					//const int y_ = y + wy;
					//const int x_ = x + wx;

					// zero if first value, otherwise shift to left
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << 1); }
					// symmetric pattern? also redundant
					*out |= (im(y-wy,x-wx) < (im(y+wy,x+wx)) ? 1 : 0);

					i += 1;
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

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};

	/** Census transform. Bits stored in row major order. */
	template<int WINX, int WINY>
	struct HCensusTransformRowMajor {
		__host__ __device__ inline void window(short center, const int y, const int x, uint64_t* __restrict__ out) {
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const uint y_ = min(uint(y + wy), hsize.y-1);
					const uint x_ = min(uint(x + wx), hsize.x-1);

					// zero if first value, otherwise shift to left
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << 1); }
					*out |= (center < (neigh(y_,x_)) ? 1 : 0);

					i += 1;
					// if all bits set, continue to next element
					if (i % 64 == 0) { out++; }
				}
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y; y<size.y; y+=stride.y) {
				for (int x = thread.x; x<size.x; x+=stride.x) {
					short center = im(y, x);
					float ny = float(y) / float(size.y) * float(hsize.y);
					float nx = float(x) / float(size.x) * float(hsize.x);
					window(center, ny, nx, &(out(y, x*WSTEP)));
				}
			}
		}

		Array2D<uchar>::Data im;
		Array2D<uchar>::Data neigh;
		Array2D<uint64_t>::Data out;
		ushort2 hsize;

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};

	/** Census transform. Bits stored in row major order.
	    @see GCensusTransform above. */
	template<int WINX, int WINY>
	struct HGCensusTransformRowMajor {
		__host__ __device__ inline void window(short2 center, ushort2 size, const int y, const int x, uint64_t* __restrict__ out) {
			uint8_t i = 0; // bit counter for *out

			for (int wy = -WINY/2; wy <= WINY/2; wy++) {
				for (int wx = -WINX/2; wx <= WINX/2; wx++) {
					const uint ny = min(uint(y + wy), hsize.y-1);
					const uint nx = min(uint(x + wx), hsize.x-1);
					const uint cy = min(uint(center.y - wy), size.y-1);
					const uint cx = min(uint(center.x - wx), size.x-1);

					// zero if first value, otherwise shift to left
					if (i % 64 == 0) { *out = 0; }
					else             { *out = (*out << 1); }
					*out |= (im(cy,cx) < (neigh(ny,nx)) ? 1 : 0);

					i += 1;
					// if all bits set, continue to next element
					if (i % 64 == 0) { out++; }
				}
			}
		}

		__host__ __device__  void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y; y<size.y; y+=stride.y) {
				for (int x = thread.x; x<size.x; x+=stride.x) {
					//short center = im(y, x);
					float ny = float(y) / float(size.y) * float(hsize.y);
					float nx = float(x) / float(size.x) * float(hsize.x);
					window({short(x),short(y)}, size, ny, nx, &(out(y, x*WSTEP)));
				}
			}
		}

		Array2D<uchar>::Data im;
		Array2D<uchar>::Data neigh;
		Array2D<uint64_t>::Data out;
		ushort2 hsize;

		// number of uint64_t values for each window
		static constexpr int WSTEP = (WINX*WINY - 1)/(sizeof(uint64_t)*8) + 1;
	};
}

void CensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	if (pattern_ == CensusPattern::STANDARD) {
		parallel2D<algorithms::CensusTransformRowMajor<9,7>>({l.data(), ct_l_.data()}, l.width, l.height);
		parallel2D<algorithms::CensusTransformRowMajor<9,7>>({r.data(), ct_r_.data()}, r.width, r.height);
	} else if (pattern_ == CensusPattern::GENERALISED) {
		parallel2D<algorithms::GCensusTransformRowMajor<9,7>>({l.data(), ct_l_.data()}, l.width, l.height);
		parallel2D<algorithms::GCensusTransformRowMajor<9,7>>({r.data(), ct_r_.data()}, r.width, r.height);
	} else {
		// TODO:
	}
}

void CensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr) {
	if (pattern_ == CensusPattern::STANDARD) {
		parallel2D<algorithms::HCensusTransformRowMajor<9,7>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
		parallel2D<algorithms::HCensusTransformRowMajor<9,7>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	} else if (pattern_ == CensusPattern::GENERALISED) {
		parallel2D<algorithms::HGCensusTransformRowMajor<9,7>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
		parallel2D<algorithms::HGCensusTransformRowMajor<9,7>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	}
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

////////////////////////////////////////////////////////////////////////////////

void ExpandingCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	parallel2D<algorithms::ExCensusTransformRowMajor<9,9>>({l.data(), ct_l_.data(),1.5f}, l.width, l.height);
	parallel2D<algorithms::ExCensusTransformRowMajor<9,9>>({r.data(), ct_r_.data(),1.5f}, r.width, r.height);
}

/*void ExpandingCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr) {
	parallel2D<algorithms::HCensusTransformRowMajor<9,7>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
	parallel2D<algorithms::HCensusTransformRowMajor<9,7>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
}*/

void ExpandingCensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
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

////////////////////////////////////////////////////////////////////////////////

void MiniCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	if (pattern_ == CensusPattern::STANDARD) {
		parallel2D<algorithms::CensusTransformRowMajor<5,3>>({l.data(), ct_l_.data()}, l.width, l.height);
		parallel2D<algorithms::CensusTransformRowMajor<5,3>>({r.data(), ct_r_.data()}, r.width, r.height);
	} else if (pattern_ == CensusPattern::GENERALISED) {
		parallel2D<algorithms::GCensusTransformRowMajor<5,3>>({l.data(), ct_l_.data()}, l.width, l.height);
		parallel2D<algorithms::GCensusTransformRowMajor<5,3>>({r.data(), ct_r_.data()}, r.width, r.height);
	} else {
		// TODO:
	}
}

void MiniCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr) {
	if (pattern_ == CensusPattern::STANDARD) {
		parallel2D<algorithms::HCensusTransformRowMajor<5,3>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
		parallel2D<algorithms::HCensusTransformRowMajor<5,3>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	} else if (pattern_ == CensusPattern::GENERALISED) {
		parallel2D<algorithms::HGCensusTransformRowMajor<5,3>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
		parallel2D<algorithms::HGCensusTransformRowMajor<5,3>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	}
}

void MiniCensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
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

////////////////////////////////////////////////////////////////////////////////

void WeightedCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r) {
	parallel2D<algorithms::CensusTransformRowMajor<7,7>>({l.data(), ct_l_.data()}, l.width, l.height);
	parallel2D<algorithms::CensusTransformRowMajor<7,7>>({r.data(), ct_r_.data()}, r.width, r.height);
}

void WeightedCensusMatchingCost::set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr) {
	//if (pattern_ == CensusPattern::STANDARD) {
		parallel2D<algorithms::HCensusTransformRowMajor<7,7>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
		parallel2D<algorithms::HCensusTransformRowMajor<7,7>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	//} else if (pattern_ == CensusPattern::GENERALISED) {
	//	parallel2D<algorithms::HGCensusTransformRowMajor<7,7>>({l.data(), hl.data(), ct_l_.data(), {ushort(hl.width), ushort(hl.height)}}, l.width, l.height);
	//	parallel2D<algorithms::HGCensusTransformRowMajor<7,7>>({r.data(), hr.data(), ct_r_.data(), {ushort(hr.width), ushort(hr.height)}}, r.width, r.height);
	//}
}

void WeightedCensusMatchingCost::set(cv::InputArray l, cv::InputArray r) {
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
