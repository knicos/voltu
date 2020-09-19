#ifndef _FTL_LIBSTEREO_FILTERS_CLUSTER_HPP_
#define _FTL_LIBSTEREO_FILTERS_CLUSTER_HPP_

#include "../util.hpp"
#include "../array1d.hpp"
#include "../array2d.hpp"
#include "../bucket1d.hpp"
#include "../bucket2d.hpp"

struct FocalCluster {
	short2 focal_pt;
	Bucket1D<short2, 64>::Data left;
	Bucket2D<ushort, 64>::Data right;
	Array1D<int>::Data histogram;

	int max_disparity = 1024;

	__device__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
		for (int y=thread.y; y<size.y; y+=stride.y) {
			int count = left(y);
			// Stride a warp of threads over the features
			for (int f=thread.x; f<count; f+=stride.x) {
				// For each feature or features near to focal point
				short2 feature = left(y,f);
				int distx = feature.x - focal_pt.x;
				
				// - For each feature in right image that matches
				const ushort *ptr = right.ptr(y, feature.y);
				int count2 = right(y,feature.y);
				for (int i=0; i<count2; ++i) {
					//   - Add focal dist to feature X and add to histogram
					int disparity = max(0,focal_pt.x - int(ptr[i]) + distx);
					if (disparity < max_disparity && disparity > 0) atomicAdd(&histogram(disparity), 1);
				}
			}
		}
	}
};

struct FocalSelector {
	short2 focal_pt;
	int focal_disp;
	Bucket1D<short2, 64>::Data left;
	Bucket2D<ushort, 64>::Data right;
	Array1D<int>::Data histogram;
	Array2D<float>::Data disparity;
	Array2D<float>::Data confidence;

	int max_disparity = 1024;

	__device__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
		for (int y=thread.y; y<size.y; y+=stride.y) {
			int count = left(y);
			// Stride a warp of threads over the features
			for (int f=thread.x; f<count; f+=stride.x) {
				// For each feature or features near to focal point
				short2 feature = left(y,f);
				int distx = feature.x - focal_pt.x;

				int best_disp = 0;
				int max_v = 0;
				
				// - For each feature in right image that matches
				const ushort *ptr = right.ptr(y, feature.y);
				int count2 = right(y,feature.y);
				for (int i=0; i<count2; ++i) {
					//   - Add focal dist to feature X and add to histogram
					int d = max(0,focal_pt.x - int(ptr[i]) + distx);
					
					int v = histogram(d);
					if (v > max_v) {
						max_v = v;
						best_disp = d;
					}
				}

				if (max_v > 0) {
					//float conf = 1.0f - min(1.0f, float(abs(best_disp-focal_disp)) / 10.0f);
					float conf = 1.0f - min(1.0f, float(abs(distx)) / 500.0f);
					if (conf > confidence(y,feature.x)) {
						disparity(y,feature.x) = float(best_disp);
						confidence(y,feature.x) = conf;
					}
				}
			}
		}
	}
};

#endif
