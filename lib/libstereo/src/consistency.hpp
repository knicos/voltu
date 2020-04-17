#ifndef _FTL_LIBSTEREO_CONSISTENCY_HPP_
#define _FTL_LIBSTEREO_CONSISTENCY_HPP_

#include "util.hpp"
#include "array2d.hpp"

namespace algorithms {

	/**
	 * Left/right disparity consistency check.
	 */
	template <typename T>
	struct ConsistencyCheck {
		typename Array2D<T>::Data left;
		const typename Array2D<T>::Data right;
		const float threshold=1.0f;
		const float value=0.0f;

		__host__ __device__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			for (int y = thread.y; y < size.y; y+=stride.y) {
				T* ptr_l = left.ptr(y);
				const T* ptr_r = right.ptr(y);

				for (int x = thread.x; x < size.x; x+=stride.x) {
					const int d = round(ptr_l[x]);

					if ((x-d) < 0 || abs(ptr_l[x] - ptr_r[x-d]) > threshold) {
						ptr_l[x] = value;
					}
				}
			}
		}
	};
}

#endif
