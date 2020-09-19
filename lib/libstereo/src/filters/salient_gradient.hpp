#ifndef _FTL_LIBSTEREO_FILTERS_SALIENT_GRADIENT_HPP_
#define _FTL_LIBSTEREO_FILTERS_SALIENT_GRADIENT_HPP_

#include "../util.hpp"
#include "../array2d.hpp"
#include "../bucket2d.hpp"
#include "../bucket1d.hpp"

/**
 * Select salient gradient features and gather into scanline buckets that
 * includes the x-coordinate and feature orientation. Not grouped by orientation.
 * 
 * TODO: This needs to be a focal point modified version. Radius from focal
 * point determines threshold, but the radius itself is determined by feature
 * density around the focal point ... ultimately keeping the number of features
 * within a reasonable very small level per scanline (say 2-8).
 */
struct SalientGradient {
	short2 focal_pt;
	int radius;
	Array2D<uchar>::Data image;
	Array2D<uchar>::Data angle;
	Array2D<uchar>::Data magnitude;
	Bucket1D<short2, 64>::Data buckets;

	int width, height;

	__cuda__ inline float2 calculateGradient(int x, int y) {
		if (x < 1 || y < 1 || x >= width-1 || y >= height-1) return make_float2(0,0);

		float dx = -1.0f*float(image(y-1,x-1)) + -2.0f*float(image(y, x-1)) + -1*float(image(y+1, x-1)) +
             float(image(y-1, x+1)) + 2.0f*float(image(y, x+1)) + float(image(y+1, x+1));
        float dy = float(image(y-1, x-1)) + 2.0f*float(image(y-1, x)) + float(image(y-1, x+1)) +
             -1.0f*float(image(y+1, x-1)) + -2.0f*float(image(y+1, x)) + -1.0f*float(image(y+1, x+1));

        float g = sqrt( (dx*dx) + (dy*dy) );
		float a = atan2(dy, dx);
		return make_float2(g,a);
	}

	struct WarpSharedMemory {
		int gradient_histogram[32];
	};

	inline __device__ int scan(volatile int *s_Data, int thread, int threshold) {
		for (uint offset = 1; offset < 32; offset <<= 1) {
			__syncwarp();
			uint t = (thread >= offset) ? s_Data[thread] + s_Data[thread - offset] : s_Data[thread];
			__syncwarp();
			s_Data[thread] = t;
		}

		uint t = __ballot_sync(0xFFFFFFFF, s_Data[thread] > threshold);
		return __ffs(t);
	}

	__device__ inline float weighting(float r, float h) {
		if (r >= h) return 0.0f;
		float rh = r / h;
		rh = 1.0f - rh*rh;
		return rh*rh*rh*rh;
	}

	__device__ inline float calcWeight(int x, int y) {
		float dx = focal_pt.x - x;
		float dy = focal_pt.y - y;
		float dist = sqrt(dx*dx + dy*dy);
		float weight = weighting(dist, float(radius));
		return weight;
	}

	__device__ void operator()(ushort2 thread, ushort2 stride, ushort2 size, WarpSharedMemory &wsm) {
		static constexpr float PI = 3.14f;
		static constexpr float PI2 = PI/2.0f;

		for (int y=thread.y; y<size.y; y+=stride.y) {
			// Reset histogram
			//for (int i=thread.x; i < 32; i+=32) wsm.gradient_histogram[i] = 0;
			wsm.gradient_histogram[thread.x] = 0;
			//int maxmag = 0;

			for (int x=thread.x; x<size.x; x+=stride.x) {
				auto g = calculateGradient(x,y);
				//float weight = calcWeight(x,y);
				angle(y,x) = uchar(min(15,int((g.y+PI2) / PI * 16.0f)));
				magnitude(y,x) = uchar(g.x);

				//maxmag = max(maxmag,int(g.x));
				atomicAdd(&wsm.gradient_histogram[min(31,int(g.x / 361.0f * 32.0f))], 1);
				//atomicAdd(&wsm.gradient_histogram[0], 1);
			}

			//maxmag = int(float(warpMax(maxmag)) * 0.8f);

			uchar gthresh = min(255, scan(wsm.gradient_histogram, thread.x, float(width)*0.95f) * (256/32));

			// Apply threshold
			for (int x=thread.x; x<size.x; x+=stride.x) {
				float weight = calcWeight(x,y);
				float thresh = gthresh; //max(gthresh, focal_thresh);
				//for (int u=-3; u<=3; ++u) {
				//	thresh = (x+u >= 0 && x+u < width) ? max(thresh, weight*float(magnitude(y,x+u))) : thresh;
				//}

				float m = float(magnitude(y,x)) * weight;
				if (m < thresh) angle(y,x) = 0;
				//output(y,x) = (m < thresh) ? 0 : 255;
				if (m >= thresh) {
					int a = angle(y,x);
					buckets.add(y, make_short2(x, a));
				}

				angle(y,x) = uchar(thresh*weight);
			}
		}
	}
};

/**
 * Find salient gradient features and gather in orientation groups scanline
 * buckets. Adds features to orientations either side of actual orientation for
 * a degree of tolerance to exact orientation. This allows fast search for
 * features based upon scanline and orientation.
 */
struct SalientGradientGrouped {
	short2 focal_pt;
	int radius;
	Array2D<uchar>::Data image;
	Array2D<uchar>::Data angle;
	Array2D<uchar>::Data magnitude;
	Bucket2D<ushort, 64>::Data buckets;

	int width, height;

	__cuda__ inline float2 calculateGradient(int x, int y) {
		if (x < 1 || y < 1 || x >= width-1 || y >= height-1) return make_float2(0,0);

		float dx = -1.0f*float(image(y-1,x-1)) + -2.0f*float(image(y, x-1)) + -1*float(image(y+1, x-1)) +
             float(image(y-1, x+1)) + 2.0f*float(image(y, x+1)) + float(image(y+1, x+1));
        float dy = float(image(y-1, x-1)) + 2.0f*float(image(y-1, x)) + float(image(y-1, x+1)) +
             -1.0f*float(image(y+1, x-1)) + -2.0f*float(image(y+1, x)) + -1.0f*float(image(y+1, x+1));

        float g = sqrt( (dx*dx) + (dy*dy) );
		float a = atan2(dy, dx);
		return make_float2(g,a);
	}

	struct WarpSharedMemory {
		int gradient_histogram[32];
	};

	inline __device__ int scan(volatile int *s_Data, int thread, int threshold) {
		for (uint offset = 1; offset < 32; offset <<= 1) {
			__syncwarp();
			uint t = (thread >= offset) ? s_Data[thread] + s_Data[thread - offset] : s_Data[thread];
			__syncwarp();
			s_Data[thread] = t;
		}

		uint t = __ballot_sync(0xFFFFFFFF, s_Data[thread] > threshold);
		return __ffs(t);
	}

	__device__ inline float weighting(float r, float h) {
		if (r >= h) return 0.0f;
		float rh = r / h;
		rh = 1.0f - rh*rh;
		return rh*rh*rh*rh;
	}

	__device__ inline float calcWeight(int x, int y) {
		float dx = focal_pt.x - x;
		float dy = focal_pt.y - y;
		float dist = sqrt(dx*dx + dy*dy);
		float weight = weighting(dist, float(radius));
		return weight;
	}

	__device__ void operator()(ushort2 thread, ushort2 stride, ushort2 size, WarpSharedMemory &wsm) {
		static constexpr float PI = 3.14f;
		static constexpr float PI2 = PI/2.0f;

		for (int y=thread.y; y<size.y; y+=stride.y) {
			// Reset histogram
			//for (int i=thread.x; i < 32; i+=32) wsm.gradient_histogram[i] = 0;
			wsm.gradient_histogram[thread.x] = 0;
			//int maxmag = 0;

			for (int x=thread.x; x<size.x; x+=stride.x) {
				auto g = calculateGradient(x,y);
				angle(y,x) = uchar(min(15,int((g.y+PI2) / PI * 16.0f)));
				magnitude(y,x) = uchar(g.x);

				//maxmag = max(maxmag,int(g.x));
				atomicAdd(&wsm.gradient_histogram[min(31,int(g.x / 361.0f * 32.0f))], 1);
				//atomicAdd(&wsm.gradient_histogram[0], 1);
			}

			//maxmag = int(float(warpMax(maxmag)) * 0.8f);

			uchar gthresh = min(255, scan(wsm.gradient_histogram, thread.x, float(width)*0.95f) * (256/32));

			// Apply threshold
			for (int x=thread.x; x<size.x; x+=stride.x) {
				float weight = calcWeight(x,y);
				float thresh = gthresh;
				//for (int u=-3; u<=3; ++u) {
				//	thresh = (x+u >= 0 && x+u < width) ? max(thresh, weight*float(magnitude(y,x+u))) : thresh;
				//}

				float m = float(magnitude(y,x))*weight;
				//if (m < thresh) angle(y,x) = 0;
				//output(y,x) = (m < thresh) ? 0 : 255;
				if (m >= thresh) {
					int a = angle(y,x);
					buckets.add(y, a, ushort(x));
					buckets.add(y, (a > 0) ? a-1 : 15, ushort(x));
					buckets.add(y, (a < 15) ? a+1 : 0, ushort(x));
				}
			}
		}
	}
};

#endif
