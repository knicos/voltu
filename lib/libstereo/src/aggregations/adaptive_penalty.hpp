#ifndef _FTL_LIBSTEREO_AGGREGATIONS_ADAPTIVE_HPP_
#define _FTL_LIBSTEREO_AGGREGATIONS_ADAPTIVE_HPP_

#include "../dsi.hpp"
#include "../array2d.hpp"

namespace ftl {
namespace stereo {
namespace aggregations {

template <typename DSIIN>
struct AdaptivePenaltySGM {
	typedef typename DSIIN::Type Type;
	typedef typename DSIIN::Type costtype_t;

	// Provided externally
	const DSIIN in;
	typename Array2D<costtype_t>::Data min_cost_all;

	const int P1;

	// Provided internally
	typename DisparitySpaceImage<costtype_t>::DataType out;
	typename DisparitySpaceImage<costtype_t>::DataType previous;
	typename DisparitySpaceImage<costtype_t>::DataType updates;
	typename Array2D<uint8_t>::Data penalties;

	struct PathData : BasePathData<costtype_t> {
		// Custom path data goes here...
		costtype_t previous_cost_min;
		costtype_t *previous;
		costtype_t *current;
	};

	struct DirectionData {
		DisparitySpaceImage<costtype_t> previous;
		DisparitySpaceImage<costtype_t> updates;
		Array2D<uint8_t> penalties;
	};

	/* Initialise buffers for a new path direction. */
	void direction(DirectionData &data, DisparitySpaceImage<costtype_t> &buffer) {
		out = buffer.data();
		data.previous.create(out.width+out.height, 1, out.disp_min, out.disp_max);
		data.updates.create(out.width+out.height, 1, out.disp_min, out.disp_max);
		previous = data.previous.data();
		updates = data.updates.data();
		data.penalties.create(out.width, out.height);  // Note: should already exist
		penalties = data.penalties.data();
	}

	/* Reset buffers to start a new path */
	__host__ __device__ inline void startPath(ushort2 pixel, ushort thread, ushort stride, PathData &data) {
		data.previous = &previous(0,data.pathix,previous.disp_min);
		data.current = &updates(0,data.pathix,updates.disp_min);

		for (int d=thread; d<=previous.disp_min; d+=stride) {
			data.previous[d] = 0;
		}

		// To ensure all threads have finished clearing the buffer
		#ifdef __CUDA_ARCH__
		__syncwarp();
		#endif
	}

	__host__ __device__ inline void endPath(ushort2 pixel, ushort thread, ushort stride, PathData &data) {}

	/* Main SGM cost aggregation function */
	__host__ __device__ inline costtype_t calculateCost(ushort2 pixel, int d, costtype_t *previous, int size, costtype_t previous_cost_min) {
		const costtype_t L_min =
			min(previous[d],
				min(costtype_t(previous_cost_min + penalties(pixel.y, pixel.x)),
						min(costtype_t(previous[min(d+1,size)] + P1),
									costtype_t(previous[max(d-1,0)] + P1))
			)
		);

		// Note: This clamping to min 0 does make a tiny difference
		auto cost = L_min + in(pixel.y,pixel.x,d+in.disp_min);
		return (cost > previous_cost_min) ? cost - previous_cost_min : 0;
	}

	/* Stride over each disparity and calculate minimum cost */
	__host__ __device__ inline void operator()(ushort2 pixel, ushort thread, ushort stride, PathData &data) {
		#ifndef __CUDA_ARCH__
		using std::min;
		using std::max;
		#endif

		const int d_stop = int(out.disp_max)-int(out.disp_min);

		costtype_t min_cost = 255;

		// For each disparity (striding the threads)
		for (int d=thread; d<=d_stop; d+=stride) {
			auto c = calculateCost(pixel, d, data.previous, d_stop, data.previous_cost_min);

			out(pixel.y,pixel.x,d+in.disp_min) += c;
			data.current[d] = c;

			// Record the smallest disparity cost for this pixel
			min_cost = (c < min_cost) ? c : min_cost;
		}

		// WARP Aggregate on GPU only (assumes stride = 1 on CPU)
		// Min cost from each thread must be combined for overall minimum
		// Each thread then obtains thread global minimum
		#ifdef __CUDA_ARCH__
		min_cost = warpMin(min_cost);
		#else
		// add assert
		#endif

		data.previous_cost_min = min_cost;
		min_cost_all(pixel.y,pixel.x) += min_cost; // atomic?

		// Swap current and previous cost buffers
		costtype_t *tmp_ptr = const_cast<costtype_t *>(data.previous);
		data.previous = data.current;
		data.current = tmp_ptr;
	}
};

}
}
}

#endif
