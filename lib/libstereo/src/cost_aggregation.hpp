#pragma once

#include "dsi.hpp"
#include "util.hpp"

#include <stereo_types.hpp>

template <typename T>
struct BasePathData {
	short2 direction;
	int pathix;
};

namespace algorithms {
	template <typename FUNCTOR, int DX, int DY>
	struct Aggregator {
		static_assert(DX != 0 || DY != 0, "DX or DY must be non-zero");
		static_assert(std::is_convertible<typename FUNCTOR::PathData, BasePathData<typename FUNCTOR::Type>>::value, "Path data must derive from BasePathData<T>");

		FUNCTOR f;

		__cuda__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
			#ifndef __CUDA_ARCH__
			using std::min;
			using std::max;
			#endif

			// TODO: Use shared memory?
			typename FUNCTOR::PathData data;
			data.direction.x = DX;
			data.direction.y = DY;

			for (int i = thread.y; i < size.y; i+=stride.y) {
				data.pathix = i;
				ushort2 pixel;

				if (DY == 0) {
					pixel.y = i;
					if (DX > 0) { pixel.x = 0; }
					else { pixel.x = f.in.width - 1; }
				}
				else if (DX == 0) {
					pixel.x = i;
					if (DY > 0) { pixel.y = 0; }
					else { pixel.y = f.in.height - 1; }
				}
				else if (DX == DY) {
					if (DX > 0) {
						pixel.x = max(0, i - f.in.height);
						pixel.y = max(0, f.in.height - i);
					}
					else {
						pixel.x = min(f.in.width - 1, f.in.width + f.in.height - i - 2);
						pixel.y = min(f.in.height - 1, i);
					}
				}
				else {
					if (DX > 0) {
						pixel.x = max(0, i - f.in.height + 1);
						pixel.y = min(f.in.height - 1, i);
					}
					else {
						pixel.x = min(i, f.in.width - 1);
						pixel.y = max(0, i - f.in.width + 1);
					}
				}

				f.startPath(pixel, thread.x, stride.x, data);

				while (pixel.x < f.in.width && pixel.y < f.in.height) {
					f(pixel, thread.x, stride.x, data);
					pixel.x += DX;
					pixel.y += DY;
				}

				f.endPath(pixel, thread.x, stride.x, data);
			}
		}
	};
}

template <typename F>
class PathAggregator {
	public:

	DisparitySpaceImage<typename F::Type> &operator()(F &f, int paths) {
		out.create(f.in.width, f.in.height, f.in.disp_min, f.in.disp_max);
		out.clear();

		for (int i=0; i<8; ++i) {
			if (paths & (1 << i)) {
				queuePath(i, f);
				//out.add(buffers[i]);
			}
		}

		return out;
	}

	typename F::DirectionData &getDirectionData(AggregationDirections d) {
		switch (d) {
		case AggregationDirections::LEFTRIGHT			: return path_data[0];
		case AggregationDirections::RIGHTLEFT			: return path_data[1];
		case AggregationDirections::UPDOWN				: return path_data[2];
		case AggregationDirections::DOWNUP				: return path_data[3];
		case AggregationDirections::TOPLEFTBOTTOMRIGHT	: return path_data[4];
		case AggregationDirections::BOTTOMRIGHTTOPLEFT	: return path_data[5];
		case AggregationDirections::BOTTOMLEFTTOPRIGHT	: return path_data[6];
		case AggregationDirections::TOPRIGHTBOTTOMLEFT	: return path_data[7];
		default: throw std::exception();
		}
	}

	private:
	DisparitySpaceImage<typename F::Type> out;
	typename F::DirectionData path_data[8];
	DisparitySpaceImage<typename F::Type> buffers[8];

	void queuePath(int id, F &f) {
		F f1 = f;
		f1.direction(path_data[id], out);

		switch (id) {
		case 0	:	parallel1DWarp<algorithms::Aggregator<F, 1, 0>>({f1}, f.in.height, f.in.disparityRange()); break;
		case 1	:	parallel1DWarp<algorithms::Aggregator<F,-1, 0>>({f1}, f.in.height, f.in.disparityRange()); break;
		case 2	:	parallel1DWarp<algorithms::Aggregator<F, 0, 1>>({f1}, f.in.width, f.in.disparityRange()); break;
		case 3	:	parallel1DWarp<algorithms::Aggregator<F, 0,-1>>({f1}, f.in.width, f.in.disparityRange()); break;
		case 4	:	parallel1DWarp<algorithms::Aggregator<F, 1, 1>>({f1}, f.in.height+f.in.width, f.in.disparityRange()); break;
		case 5	:	parallel1DWarp<algorithms::Aggregator<F,-1,-1>>({f1}, f.in.height+f.in.width, f.in.disparityRange()); break;
		case 6	:	parallel1DWarp<algorithms::Aggregator<F, 1,-1>>({f1}, f.in.height+f.in.width, f.in.disparityRange()); break;
		case 7	:	parallel1DWarp<algorithms::Aggregator<F,-1, 1>>({f1}, f.in.height+f.in.width, f.in.disparityRange()); break;
		}
	}
};
