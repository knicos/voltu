#include <ftl/operators/cuda/mls/multi_intensity.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <ftl/cuda/weighting.hpp>

using ftl::cuda::MLSMultiIntensity;
using cv::cuda::GpuMat;

// ==== Multi image MLS ========================================================

__device__ inline float featureWeight(int f1, int f2) {
	const float w = (1.0f-(float(abs(f1 - f2)) / 255.0f));
	return w*w*w;
}

/*
 * Gather points for Moving Least Squares, from each source image
 */
 template <int SEARCH_RADIUS, typename T>
 __global__ void mls_gather_intensity_kernel(
	const half4* __restrict__ normals_in,
	half4* __restrict__ normals_out,
	const float* __restrict__ depth_origin,
	const float* __restrict__ depth_in,
	const T* __restrict__ feature_origin,
	const T* __restrict__ feature_in,
	float4* __restrict__ centroid_out,
	float* __restrict__ contrib_out,
	float smoothing,
	float fsmoothing,
	float4x4 o_2_in,
	float4x4 in_2_o,
	float3x3 in_2_o33,
	ftl::rgbd::Camera camera_origin,
	ftl::rgbd::Camera camera_in,
	int npitch_out,
	int cpitch_out,
	int wpitch_out,
	int dpitch_o,
	int dpitch_i,
	int npitch_in,
	int fpitch_o,
	int fpitch_i
) {        
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= camera_origin.width || y >= camera_origin.height) return;

	float3 nX = make_float3(normals_out[y*npitch_out+x]);
	float3 aX = make_float3(centroid_out[y*cpitch_out+x]);
    float contrib = contrib_out[y*wpitch_out+x];

	float d0 = depth_origin[x+y*dpitch_o];
	if (d0 <= camera_origin.minDepth || d0 >= camera_origin.maxDepth) return;

	const uchar2 feature1 = feature_origin[x+y*fpitch_o];

	float3 X = camera_origin.screenToCam((int)(x),(int)(y),d0);

	int2 s = camera_in.camToScreen<int2>(o_2_in * X);

	// TODO: Could dynamically adjust the smoothing factors depending upon the
	// number of matches. Meaning, if lots of good local and feature matches
	// then be less likely to include poorer matches. Conversely, if only poor
	// non-local or feature distance matches, then increase search range.

    // Neighbourhood
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
		const float d = (s.x+u >= 0 && s.x+u < camera_in.width && s.y+v >= 0 && s.y+v < camera_in.height) ? depth_in[s.x+u+(s.y+v)*dpitch_i] : 0.0f;
		if (d <= camera_in.minDepth || d >= camera_in.maxDepth) continue;

		// Point and normal of neighbour
		const float3 Xi = in_2_o * camera_in.screenToCam(s.x+u, s.y+v, d);
		const float3 Ni = make_float3(normals_in[s.x+u+(s.y+v)*npitch_in]);

		const uchar2 feature2 = feature_in[s.x+y+(s.y+v)*fpitch_i];

		// Gauss approx weighting functions
		// Rule: spatially close and feature close is strong
		// Spatially far or feature far, then poor.
		// So take the minimum, must be close and feature close to get good value
		const float w_high_int = ftl::cuda::weighting(float(abs(int(feature1.x)-int(feature2.x))), fsmoothing);
		const float w_mean_int = ftl::cuda::weighting(float(abs(int(feature1.y)-int(feature2.y))), 100.0f);
		const float w_space = ftl::cuda::spatialWeighting(X,Xi,smoothing); 
		// TODO: Distance from cam squared
		// TODO: Angle from cam (dot of normal and ray)
		const float w = (length(Ni) > 0.0f)
			? min(w_space, min(w_high_int, w_mean_int))
			: 0.0f;

		aX += Xi*w;
		nX += (in_2_o33 * Ni)*w;
		contrib += w;
    }
	}

	normals_out[y*npitch_out+x] = make_half4(nX, 0.0f);
	centroid_out[y*cpitch_out+x] = make_float4(aX, 0.0f);
	contrib_out[y*wpitch_out+x] = contrib;
}

/**
 * Convert accumulated values into estimate of depth and normals at pixel.
 */
 __global__ void mls_reduce_kernel_2(
	const float4* __restrict__ centroid,
	const half4* __restrict__ normals,
	const float* __restrict__ contrib_out,
	half4* __restrict__ normals_out,
	float* __restrict__ depth,
	ftl::rgbd::Camera camera,
	int npitch_in,
	int cpitch_in,
	int wpitch,
	int npitch,
	int dpitch
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < camera.width && y < camera.height) {
		float3 nX = make_float3(normals[y*npitch_in+x]);
		float3 aX = make_float3(centroid[y*cpitch_in+x]);
		float contrib = contrib_out[y*wpitch+x];

		//depth[x+y*dpitch] = X.z;
		normals_out[x+y*npitch] = make_half4(0.0f, 0.0f, 0.0f, 0.0f);

		float d0 = depth[x+y*dpitch];
		//depth[x+y*dpitch] = 0.0f;
		if (d0 <= camera.minDepth || d0 >= camera.maxDepth || contrib == 0.0f) return;
		float3 X = camera.screenToCam((int)(x),(int)(y),d0);
		
		nX /= contrib;  // Weighted average normal
		aX /= contrib;  // Weighted average point (centroid)

		// Signed-Distance Field function
		float fX = nX.x * (X.x - aX.x) + nX.y * (X.y - aX.y) + nX.z * (X.z - aX.z);

		// Calculate new point using SDF function to adjust depth (and position)
		X = X - nX * fX;

		depth[x+y*dpitch] = X.z;
		normals_out[x+y*npitch] = make_half4(nX / length(nX), 0.0f);
	}
}


MLSMultiIntensity::MLSMultiIntensity(int radius)
	: radius_(radius)
{

}

MLSMultiIntensity::~MLSMultiIntensity()
{

}

void MLSMultiIntensity::prime(
	const GpuMat &depth_prime,
	const GpuMat &intensity_prime,
	const ftl::rgbd::Camera &cam_prime,
	const float4x4 &pose_prime,
	cudaStream_t stream)
{
	depth_prime_ = depth_prime;
	intensity_prime_ = intensity_prime;
	cam_prime_ = cam_prime;
	pose_prime_ = pose_prime;

	centroid_accum_.create(depth_prime.size(), CV_32FC4);
	normal_accum_.create(depth_prime.size(), CV_16FC4);
	weight_accum_.create(depth_prime.size(), CV_32F);

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	// Reset buffers
	centroid_accum_.setTo(cv::Scalar(0,0,0,0), cvstream);
	weight_accum_.setTo(cv::Scalar(0), cvstream);
	normal_accum_.setTo(cv::Scalar(0,0,0,0), cvstream);
}

void MLSMultiIntensity::gatherPrime(float smoothing, cudaStream_t stream)
{
	// Can use a simpler kernel without pose transformations
}

void MLSMultiIntensity::gather(
	const GpuMat &depth_src,
	const GpuMat &normals_src,
	const GpuMat &intensity_src,
	const ftl::rgbd::Camera &cam_src,
	const float4x4 &pose_src,
	float smoothing,
	float fsmoothing,
	cudaStream_t stream)
{
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_prime_.cols + THREADS_X - 1)/THREADS_X, (depth_prime_.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	float4x4 inv_pose_src = pose_src;
	inv_pose_src.invert();
	float4x4 o_2_in = inv_pose_src * pose_prime_;
	float4x4 inv_pose_prime = pose_prime_;
	inv_pose_prime.invert();
	float4x4 in_2_o = inv_pose_prime * pose_src;
	float3x3 in_2_o33 = inv_pose_prime.getFloat3x3() * pose_src.getFloat3x3();

	mls_gather_intensity_kernel<3><<<gridSize, blockSize, 0, stream>>>(
		normals_src.ptr<half4>(),
		normal_accum_.ptr<half4>(),
		depth_prime_.ptr<float>(),
		depth_src.ptr<float>(),
		intensity_prime_.ptr<uchar2>(),
		intensity_src.ptr<uchar2>(),
		centroid_accum_.ptr<float4>(),
		weight_accum_.ptr<float>(),
		smoothing,
		fsmoothing,
		o_2_in,
		in_2_o,
		in_2_o33,
		cam_prime_,
		cam_src,
		normal_accum_.step1()/4,
		centroid_accum_.step1()/4,
		weight_accum_.step1(),
		depth_prime_.step1(),
		depth_src.step1(),
		normals_src.step1()/4,
		intensity_prime_.step1()/2,
		intensity_src.step1()/2
	);
	cudaSafeCall( cudaGetLastError() );
}

void MLSMultiIntensity::adjust(
	GpuMat &depth_out,
	GpuMat &normals_out,
	cudaStream_t stream)
{
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((depth_prime_.cols + THREADS_X - 1)/THREADS_X, (depth_prime_.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	normals_out.create(depth_prime_.size(), CV_16FC4);
	depth_out.create(depth_prime_.size(), CV_32F);

	// FIXME: Depth prime assumed to be same as depth out

	mls_reduce_kernel_2<<<gridSize, blockSize, 0, stream>>>(
		centroid_accum_.ptr<float4>(),
		normal_accum_.ptr<half4>(),
		weight_accum_.ptr<float>(),
		normals_out.ptr<half4>(),
		depth_prime_.ptr<float>(),
		cam_prime_,
		normal_accum_.step1()/4,
		centroid_accum_.step1()/4,
		weight_accum_.step1(),
		normals_out.step1()/4,
		depth_prime_.step1()
	);
	cudaSafeCall( cudaGetLastError() );
}

// =============================================================================

template <int RADIUS>
__global__ void mean_subtract_kernel(
	const uchar* __restrict__ intensity,
	uchar2* __restrict__ contrast,
	int pitch,
	int cpitch,
	int width,
	int height
) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= RADIUS && y >= RADIUS && x < width-RADIUS && y < height-RADIUS) {
		float mean = 0.0f;

		for (int v=-RADIUS; v<=RADIUS; ++v) {
		for (int u=-RADIUS; u<=RADIUS; ++u) {
			mean += float(intensity[x+u+(y+v)*pitch]);
		}
		}

		mean /= float((2*RADIUS+1)*(2*RADIUS+1));

		float diff = float(intensity[x+y*pitch]) - mean;
		contrast[x+y*pitch] = make_uchar2(max(0, min(254, int(diff)+127)), int(mean));
	}
}

void ftl::cuda::mean_subtract(
	const cv::cuda::GpuMat &intensity,
	cv::cuda::GpuMat &contrast,
	int radius,
	cudaStream_t stream
) {
	static constexpr int THREADS_X = 8;
	static constexpr int THREADS_Y = 8;

	const dim3 gridSize((intensity.cols + THREADS_X - 1)/THREADS_X, (intensity.rows + THREADS_Y - 1)/THREADS_Y);
	const dim3 blockSize(THREADS_X, THREADS_Y);

	contrast.create(intensity.size(), CV_8UC2);

	mean_subtract_kernel<3><<<gridSize, blockSize, 0, stream>>>(
		intensity.ptr<uchar>(),
		contrast.ptr<uchar2>(),
		intensity.step1(),
		contrast.step1()/2,
		intensity.cols,
		intensity.rows
	);
	cudaSafeCall( cudaGetLastError() );
}
