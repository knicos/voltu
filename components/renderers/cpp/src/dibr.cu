#include <ftl/render/render_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/makers.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::TextureObject;
using ftl::render::Parameters;
using ftl::rgbd::Projection;

/*
 * DIBR point cloud with a depth check
 */
 __global__ void dibr_merge_kernel(TextureObject<float> depth,
		TextureObject<int> depth_out,
		float4x4 transform,
		ftl::rgbd::Camera cam,
		Parameters params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d0 = depth.tex2D(x, y);
	if (d0 <= cam.minDepth || d0 >= cam.maxDepth) return;

	const float3 camPos = transform * cam.screenToCam(x,y,d0);

	//const float d = camPos.z;

	//const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);
	const float3 screenPos = params.camera.project<Projection::PERSPECTIVE>(camPos);
	const unsigned int cx = (unsigned int)(screenPos.x+0.5f);
	const unsigned int cy = (unsigned int)(screenPos.y+0.5f);
	const float d = screenPos.z;
	if (d > params.camera.minDepth && d < params.camera.maxDepth && cx < depth_out.width() && cy < depth_out.height()) {
		// Transform estimated point to virtual cam space and output z
		atomicMin(&depth_out(cx,cy), d * 100000.0f);
	}
}

/*
* DIBR Point cloud with a constant depth assumption
*/
__global__ void dibr_merge_kernel(
		TextureObject<int> depth_out,
		float4x4 transform,
		ftl::rgbd::Camera cam,
		Parameters params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d0 = 1.0f;

	const float3 camPos = transform * cam.screenToCam(x,y,d0);
	const float d = camPos.z;

	const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);
	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;
	if (d > params.camera.minDepth && d < params.camera.maxDepth && cx < depth_out.width() && cy < depth_out.height()) {
		// Transform estimated point to virtual cam space and output z
		atomicMin(&depth_out(cx,cy), d * 100000.0f);
	}
}


void ftl::cuda::dibr_merge(TextureObject<float> &depth, TextureObject<int> &depth_out, const float4x4 &transform, const ftl::rgbd::Camera &cam, Parameters params, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	dibr_merge_kernel<<<gridSize, blockSize, 0, stream>>>(depth, depth_out, transform, cam, params);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::dibr_merge(TextureObject<int> &depth_out, const float4x4 &transform, const ftl::rgbd::Camera &cam, Parameters params, cudaStream_t stream) {
    const dim3 gridSize((cam.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (cam.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	dibr_merge_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, transform, cam, params);
    cudaSafeCall( cudaGetLastError() );
}

// ==== Normalize ==============================================================

template <typename A, typename B, bool FLIPY>
__global__ void dibr_normalise_kernel(
        TextureObject<A> in,
        TextureObject<B> out,
        TextureObject<int> contribs) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < in.width() && y < in.height()) {
		const float contrib = float(contribs.tex2D((int)x,(int)y) & 0xFFFFFF) / float(0xFFFF);
        const A a = in.tex2D((int)x,(int)y);
        //const float4 normal = normals.tex2D((int)x,(int)y);

		//out(x,y) = (contrib == 0.0f) ? make<B>(a) : make<B>(a / contrib);

        if (contrib > 0.0f) {
            if (FLIPY) out(x,out.height()-y-1) = make<B>(a / contrib);
            else out(x,y) = make<B>(a / contrib);
            //normals(x,y) = normal / contrib;
        }
    }
}

template <typename A, typename B>
void ftl::cuda::dibr_normalise(TextureObject<A> &in, TextureObject<B> &out, TextureObject<int> &contribs, bool flip, cudaStream_t stream) {
    const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    if (flip) {
        dibr_normalise_kernel<A,B,true><<<gridSize, blockSize, 0, stream>>>(in, out, contribs);
    } else {
        dibr_normalise_kernel<A,B,false><<<gridSize, blockSize, 0, stream>>>(in, out, contribs);
    }
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::dibr_normalise<float4,uchar4>(TextureObject<float4> &in, TextureObject<uchar4> &out, TextureObject<int> &contribs, bool, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float,float>(TextureObject<float> &in, TextureObject<float> &out, TextureObject<int> &contribs, bool, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float4,float4>(TextureObject<float4> &in, TextureObject<float4> &out, TextureObject<int> &contribs, bool, cudaStream_t stream);

// Float version

template <typename A, typename B>
__global__ void dibr_normalise_kernel(
        TextureObject<A> in,
        TextureObject<B> out,
        TextureObject<float> weights) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < in.width() && y < in.height()) {
		const float contrib = weights.tex2D((int)x,(int)y);
        const A a = in.tex2D((int)x,(int)y);

        if (contrib > 0.0f) {
            out(x,y) = make<B>(a / contrib);
        }
    }
}

template <typename A, typename B>
void ftl::cuda::dibr_normalise(TextureObject<A> &in, TextureObject<B> &out, TextureObject<float> &weights, cudaStream_t stream) {
    const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(in, out, weights);
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::dibr_normalise<float4,uchar4>(TextureObject<float4> &in, TextureObject<uchar4> &out, TextureObject<float> &weights, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float,float>(TextureObject<float> &in, TextureObject<float> &out, TextureObject<float> &weights, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float4,float4>(TextureObject<float4> &in, TextureObject<float4> &out, TextureObject<float> &weights, cudaStream_t stream);
