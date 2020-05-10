#pragma once
#include "array2d.hpp"
#include "util.hpp"
#include "consistency.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

namespace algorithms {
	template<typename T>
	__cuda__ inline float subpixel_estimate_parabola(const T &dsi, const int x, const int y, const int d) {
		if (d == dsi.disp_max || d == dsi.disp_min) { return d; }

		const float val_l = dsi(y,x,d-1);
		const float val_r = dsi(y,x,d+1);
		const float val = dsi(y,x,d);
		const float a = val_r - val_l;
		const float b = 2.0f * val - val_l - val_r;
		if (b != 0.0f) { return d + 0.5f *a/b; }
		else { return d; }
	}
	template<typename T>
	__cuda__ inline float subpixel_estimate_equiangular(const T &dsi, const int x, const int y, const int d) {
		#ifndef __CUDA_ARCH__
		using std::max;
		#endif

		if (d == dsi.disp_max || d == dsi.disp_min) { return d; }

		const float val_l = dsi(y,x,d-1);
		const float val_r = dsi(y,x,d+1);
		const float val = dsi(y,x,d);
		const float a = val_r - val_l;
		const float b = 2.0f * (val - max(val_l, val_r));
		if (b != 0.0f) { return d + a/b; }
		else { return d; }
	}

	template<typename T>
	__cuda__ inline float subpixel_estimate_sortsgm(const T &dsi, const int x, const int y, const int d) {
		if (d == dsi.disp_max || d == dsi.disp_min) { return d; }

		/**
		* Eq. 20 in
		*
		* Pantilie, C. D., & Nedevschi, S. (2012). SORT-SGM: Subpixel optimized
		* real-time semiglobal matching for intelligent vehicles. IEEE Transactions
		* on Vehicular Technology. https://doi.org/10.1109/TVT.2012.2186836
		*/

		const float C_l = dsi(y,x,d-1);
		const float C_r = dsi(y,x,d+1);
		const float C = dsi(y,x,d);

		const float delta_l = C_l - C;
		const float delta_r = C_r - C;

		const float rho = (delta_l <= delta_r) ? delta_l/delta_r : delta_r/delta_l;
		const float g = (sin((M_PI/2.0f)*(rho - 1.0f)) + 1.0f)/2.0f;

		if (delta_l <= delta_r)	{ return float(d) - 0.5f + g; }
		else					{ return float(d) + 0.5f - g; }
	}

	template <typename DSI, typename TDisp, int SUBPIXEL>
	struct WTA {
		const typename DSI::DataType dsi;
		typename Array2D<TDisp>::Data disparity;
		typename Array2D<typename DSI::Type>::Data min_cost;

		__cuda__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			using TCost = typename DSI::Type;

			for (int y = thread.y; y < size.y; y+=stride.y) {
				auto* ptr_disparity = disparity.ptr(y);
				auto* ptr_min_cost = min_cost.ptr(y);

				for (int x = 0; x < dsi.width; ++x) {
					TCost min1 = std::numeric_limits<TCost>::max();
					//TCost min2 = TCost(-1); //std::numeric_limits<TCost>::max();
					TDisp d = TDisp(0);

					for (int d_ = thread.x + dsi.disp_min; d_ <= dsi.disp_max; d_+=stride.x) {
						const TCost val = dsi(y,x,d_);

						if (val < min1) {
							//min2 = min1;
							min1 = val;
							d = d_;
						}

						// next best local minima
						/*if ((d_ != dsi.disp_min && d != dsi.disp_max) &&
							(dsi(y,x,d_-1) > val && val < dsi(y,x,d_+1)) &&
							(d != d_) && (val < min2)) {

							min2 = val;
						}*/

					}

					#ifdef __CUDA_ARCH__
					TCost mincost = warpMin(min1);
					if (min1 != mincost) d = -1;
					// FIXME: Possible non-determinism, need to choose
					#endif

					/*if (dsi(y, x, dsi.disp_max) < min2) {
						min2 = dsi(y, x, dsi.disp_max);
					}*/

					if (d >= 0) {
						if (SUBPIXEL == 1) {
							d = subpixel_estimate_parabola(dsi, x, y, d);
						}
						else if (SUBPIXEL == 2) {
							d = subpixel_estimate_equiangular(dsi, x, y, d);
						}
						else if (SUBPIXEL == 3) {
							d = subpixel_estimate_sortsgm(dsi, x, y, d);
						}

						ptr_disparity[x] = d;
						ptr_min_cost[x] =  min1;
					}
				}
			}
		}
	};

	/**
	 * Diagonal WTA (right disparity). No subpixel estimation.
	 */
	template <typename DSI, typename TDisp>
	struct WTADiagonal {
		const typename DSI::DataType dsi;
		typename Array2D<TDisp>::Data disparity;

		__cuda__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			using TCost = typename DSI::Type;
			#ifndef __CUDA_ARCH__
			using std::min;
			#endif

			for (int y = thread.y; y < size.y; y+=stride.y) {
				TDisp* ptr_disparity = disparity.ptr(y);

				for (int x = thread.x; x < size.x; x+=stride.x) {
					TCost min1 = std::numeric_limits<TCost>::max();
					TDisp d = TDisp(0);

					for (int d_ = dsi.disp_min; d_ < min(dsi.disp_max+1, dsi.width-x); d_++) {
						const TCost val = dsi(y,x+d_,d_);

						if (val < min1) {
							min1 = val;
							d = d_;
						}
					}

					ptr_disparity[x] = d;
				}
			}
		}
	};
}

template<typename DSI, typename TDisp=float>
struct WinnerTakesAll {
	Array2D<TDisp> disparity;
	Array2D<TDisp> disparity_right;
	Array2D<typename DSI::Type> min_cost;

	Array2D<TDisp> &operator()(const DSI& dsi, const int subpixel=0, const bool lr_consistency=true) {
		disparity.create(dsi.width(), dsi.height());
		disparity_right.create(dsi.width(), dsi.height());
		min_cost.create(dsi.width(), dsi.height());

		if (subpixel == 0) {
			algorithms::WTA<DSI,float,0> wta = {dsi.data(), disparity.data(), min_cost.data()};
			parallel1DWarp(wta, dsi.height(), dsi.numDisparities());
		} else if (subpixel == 1) {
			algorithms::WTA<DSI,float,1> wta = {dsi.data(), disparity.data(), min_cost.data()};
			parallel1DWarp(wta, dsi.height(), dsi.numDisparities());
		} else if (subpixel == 2) {
			algorithms::WTA<DSI,float,2> wta = {dsi.data(), disparity.data(), min_cost.data()};
			parallel1DWarp(wta, dsi.height(), dsi.numDisparities());
		} else if (subpixel == 3) {
			algorithms::WTA<DSI,float,3> wta = {dsi.data(), disparity.data(), min_cost.data()};
			parallel1DWarp(wta, dsi.height(), dsi.numDisparities());
		}

		algorithms::WTADiagonal<DSI,float> wtadiag = {dsi.data(), disparity_right.data()};
		parallel2D(wtadiag, dsi.width(), dsi.height());

		if (lr_consistency) {
			parallel2D<algorithms::ConsistencyCheck<float>>({disparity.data(), disparity_right.data()}, dsi.width(), dsi.height());
		}
		return disparity;
	}
};
