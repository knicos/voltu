#include <ftl/disparity/features.hpp>
#include <ftl/cuda_common.hpp>
#include <cudatl/colours.hpp>
#include <opencv2/cudaimgproc.hpp>

using ftl::disparity::ColourFeatures;

ColourFeatures::ColourFeatures() {

}

ColourFeatures::~ColourFeatures() {

}

inline __device__ float absmax(float a, float b, float c) {
	const float aa = fabsf(a);
	const float ab = fabsf(b);
	const float ac = fabsf(c);
	if (aa >= ab && aa >= ac) return a;
	if (ab >= aa && ab >= ac) return b;
	if (ac >= aa && ac >= ab) return c;
	return 0.0f;
}

template <int RADIUS>
__global__ void colour_features_kernel(
	const uchar4* __restrict__ image,
	int image_pitch,
	int width, int height,
	uchar* __restrict__ sig,
	uchar* __restrict__ category,
	int pitch
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	static constexpr float PIXEL_COUNT_I = ((2*RADIUS+1)*(2*RADIUS+1));
	static constexpr float PIXEL_COUNT = float(PIXEL_COUNT_I);

	if (x >= RADIUS && y >= RADIUS && x < width-RADIUS && y < height-RADIUS) {
		uchar4 c = image[x+y*image_pitch];
		int maxR = 0;
		int maxG = 0;
		int maxB = 0;

		// First, find greatest difference of immediate surroundings.
		for (int v=-1; v<=1; ++v) {
			#pragma unroll
			for (int u=-1; u<=1; ++u) {
				uchar4 cN = image[x+u+(y+v)*image_pitch];
				maxR = max(maxR, abs(int(cN.z) - int(c.z)));
				maxG = max(maxG, abs(int(cN.y) - int(c.y)));
				maxB = max(maxB, abs(int(cN.x) - int(c.x)));
			}
		}

		int match_count_r = 0;
		int match_count_g = 0;
		int match_count_b = 0;
		float match_r_val = 0.0f;
		float nonmatch_r_val = 0.0f;
		float match_g_val = 0.0f;
		float nonmatch_g_val = 0.0f;
		float match_b_val = 0.0f;
		float nonmatch_b_val = 0.0f;

		for (int v=-RADIUS; v<=RADIUS; ++v) {
			for (int u=-RADIUS; u<=RADIUS; ++u) {
				uchar4 cN = image[x+u+(y+v)*image_pitch];
				if (abs(int(cN.z) - int(c.z)) < maxR) {
					++match_count_r;
					match_r_val += cN.z;
				} else {
					nonmatch_r_val += cN.z;
				}
				if (abs(int(cN.y) - int(c.y)) < maxG) {
					++match_count_g;
					match_g_val += cN.y;
				} else {
					nonmatch_g_val += cN.y;
				}
				if (abs(int(cN.x) - int(c.x)) < maxB) {
					++match_count_b;
					match_b_val += cN.x;
				} else {
					nonmatch_b_val += cN.x;
				}
			}
		}

		match_r_val /= match_count_r;
		nonmatch_r_val /= PIXEL_COUNT_I - match_count_r;
		match_g_val /= match_count_g;
		nonmatch_g_val /= PIXEL_COUNT_I - match_count_g;
		match_b_val /= match_count_b;
		nonmatch_b_val /= PIXEL_COUNT_I - match_count_b;

		float sim_r = (fabsf(float(c.z) - match_r_val) / 255.0f);
		float diff_r = fabsf(match_r_val - nonmatch_r_val) / 255.0f;
		float sig_r = fabsf(float(match_count_r) / PIXEL_COUNT - 0.5f)*2.0f;
		sig_r = 1.0f - sig_r;
		sig_r *= 1.0f - sim_r;
		//sig_r *= diff_r;
		//sig_r = (1.0f - sim_r)*diff_r;
		//sig_r *= min(1.0f, (float(maxR) / 60.0f));

		float sim_g = (fabsf(float(c.y) - match_g_val) / 255.0f);
		float diff_g = fabsf(match_g_val - nonmatch_g_val) / 255.0f;
		float sig_g = fabsf(float(match_count_g) / PIXEL_COUNT - 0.5f)*2.0f;
		sig_g = 1.0f - sig_g;
		sig_g *= 1.0f - sim_g;
		//sig_g *= diff_g;
		//sig_g = (1.0f - sim_g)*diff_g;
		//sig_g *= min(1.0f, (float(maxG) / 60.0f));

		float sim_b = (fabsf(float(c.x) - match_b_val) / 255.0f);
		float diff_b = fabsf(match_b_val - nonmatch_b_val) / 255.0f;
		float sig_b = fabsf(float(match_count_b) / PIXEL_COUNT - 0.5f)*2.0f;
		sig_b = 1.0f - sig_b;
		sig_b *= 1.0f - sim_b;
		//sig_b *= diff_b;
		//sig_b = (1.0f - sim_r)*diff_b;
		//sig_b *= min(1.0f, (float(maxB) / 60.0f));

		uchar3 hsv = cudatl::rgb2hsv(match_r_val, match_g_val, match_b_val);
		category[x+y*pitch] = hsv.x; //0.2126f * match_r_val + 0.7152f * match_g_val + 0.0722f * match_b_val;

		if (match_r_val < nonmatch_r_val) sig_r = -sig_r;
		if (match_g_val < nonmatch_g_val) sig_g = -sig_g;
		if (match_b_val < nonmatch_b_val) sig_b = -sig_b;
		const float msig = absmax(sig_r, sig_g, sig_b);
		sig[x+y*pitch] = char(msig * 127.0f);
	}
}

__global__ void thin_features_kernel(
	uchar* __restrict__ sig,
	int pitch,
	int width, int height
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 1 && y >= 1 && x < width-1 && y < height-1) {
		const char nP = sig[x-1+y*pitch];
		const char n = sig[x+y*pitch];
		const char nN = sig[x+1+y*pitch];

		uchar v = 0;
		if ((nP < 0 && n > 0) || (nP > 0 && n < 0)) v = max(n, nN);
		else if ((nN > 0 && n < 0) || (nN < 0 && n > 0)) v = max(n, nP);

		sig[x+y*pitch] = v;
	}
}

void ColourFeatures::generate(
	const cv::cuda::GpuMat &image,
	cudaStream_t stream
) {
	cv::cuda::cvtColor(image, hls_, cv::COLOR_BGR2Lab, 4);
	sig_.create(image.size(), CV_8UC1);
	category_.create(image.size(), CV_8UC1);

	static constexpr int THREADS_X = 16;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((image.cols + THREADS_X - 1)/THREADS_X, (image.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	colour_features_kernel<3><<<gridSize, blockSize, 0, stream>>>(
		hls_.ptr<uchar4>(),
		image.step1() / 4,
		image.cols, image.rows,
		sig_.ptr<uchar>(),
		category_.ptr<uchar>(),
		sig_.step1()
	);

	printLastCudaError("Generating features error");

	thin_features_kernel<<<gridSize, blockSize, 0, stream>>>(
		sig_.ptr<uchar>(),
		sig_.step1(),
		sig_.cols, sig_.rows
	);

	printLastCudaError("Thin features error");
}

__global__ void vis_colour_features(
	const uchar* __restrict__ sig,
	const uchar* __restrict__ category,
	int pitch,
	uchar4* __restrict__ out,
	int out_pitch,
	int width, int height,
	ColourFeatures::Feature feature,
	int threshold
) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < width && y < height) {
		int s = char(sig[x+y*pitch]);
		const uchar c = category[x+y*pitch];

		/*s = -s;

		out[x+y*out_pitch] = (s >= 0) ? make_uchar4(
			(c & 0x04) ? s*2 : 0,
			(c & 0x02) ? s*2 : 0,
			(c & 0x01) ? s*2 : 0,
			255
		) : make_uchar4(0,0,0,0);*/

		//s = min(255, s*4);

		out[x+y*out_pitch] = make_uchar4(0,0,0,0);
		//if (abs(s) >= 2) {
			//uchar3 rgb = cudatl::hsv2rgb(c, uchar(255), uchar(abs(s*2)));
			//out[x+y*out_pitch] = make_uchar4(rgb.z, rgb.y, rgb.x, 255);

			out[x+y*out_pitch] = (s >= 0) ? make_uchar4(
				s*2, 0, 0, 255
			) : make_uchar4(0,0,-s*2,255);
		//}
		/*if (abs(s) > 1) {
			out[x+y*out_pitch] = (s > 0) ? make_uchar4(
				0, c, 0, 255
			) : make_uchar4(0,0,c,255);
		}*/
	}
}

void ColourFeatures::visualise(
	ColourFeatures::Feature f,
	int threshold,
	cv::cuda::GpuMat &out,
	cudaStream_t stream
) {
	out.create(sig_.size(), CV_8UC4);

	static constexpr int THREADS_X = 16;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((out.cols + THREADS_X - 1)/THREADS_X, (out.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	vis_colour_features<<<gridSize, blockSize, 0, stream>>>(
		sig_.ptr<uchar>(),
		category_.ptr<uchar>(),
		sig_.step1(),
		out.ptr<uchar4>(),
		out.step1()/4,
		out.cols, out.rows,
		f,
		threshold
	);

	printLastCudaError("Visualising features error");
}
