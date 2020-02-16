#include "weighting_cuda.hpp"
#include <ftl/cuda/weighting.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::Mask;

__device__ inline bool isValid(const ftl::rgbd::Camera &camera, const float3 &d) {
	return d.z >= camera.minDepth && d.z <= camera.maxDepth;
}

__device__ inline float square(float v) { return v*v; }

__global__ void pixel_weight_kernel(
		ftl::cuda::TextureObject<short> weight_out,
		ftl::cuda::TextureObject<uint8_t> mask_out,
		ftl::cuda::TextureObject<half4> normals_out,
		ftl::cuda::TextureObject<uchar4> support,
		ftl::cuda::TextureObject<float> depth, 
		ftl::rgbd::Camera camera,
		const cv::Size size, ftl::cuda::PixelWeightingParameters params) {

	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < size.width && y < size.height) {
		Mask mask(0);

		const float d = depth.tex2D((int)x, (int)y);
		// Multiples of pixel size at given depth
		//const float threshold = (depthCoef / ((depthCoef / d) - (radius+disconDisparities-1))) - d;
		const float threshold = params.depthCoef * d;  // Where depthCoef = 1 / focal * N, N = number of pixel distances equal to a discon.

		float weight = 0.0f;

		if (d > camera.minDepth && d < camera.maxDepth) {
			if (params.depth) {
				weight = 1.0f - ((d - camera.minDepth) / (camera.maxDepth - camera.minDepth));
				weight *= weight;
			} else {
				weight = 1.0f;
			}
			/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time.
			 * This paper just says to remove values around discontinuities. */

			const float depths[4] = {
				depth.tex2D((int)x+0, (int)y+1),
				depth.tex2D((int)x+1, (int)y+0),
				depth.tex2D((int)x+0, (int)y-1),
				depth.tex2D((int)x-1, (int)y+0)
			};

			if (params.normals) {
				const float3 CC = camera.screenToCam(x+0, y+0, d);
				const float3 PC = camera.screenToCam(x+0, y+1, depths[0]);
				const float3 CP = camera.screenToCam(x+1, y+0, depths[1]);
				const float3 MC = camera.screenToCam(x+0, y-1, depths[2]);
				const float3 CM = camera.screenToCam(x-1, y+0, depths[3]);

				//if(CC.z <  && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF) {
				if (isValid(camera,CC) && isValid(camera,PC) && isValid(camera,CP) && isValid(camera,MC) && isValid(camera,CM)) {
					float3 n = cross(PC-MC, CP-CM);
					const float  l = length(n);

					if(l > 0.0f) {
						n = n / -l;
						if (normals_out.isValid()) normals_out(x,y) = make_half4(n, 0.0f);
						float3 ray = camera.screenToCam(x, y, d);
						ray = ray / length(ray);
						weight *= min(1.0f, 1.4*dot(ray,n));
					}
				}
			}

			// Find max change in depth gradient in each direction
			const float g1 = fabsf((depths[3] - d) - (d - depths[1]));
			const float g2 = fabsf((depths[2] - d) - (d - depths[0]));
			const float g3 = fabsf((depth.tex2D(x-1, y-1) - d) - (d - depth.tex2D(x+1,y+1)));
			const float g4 = fabsf((depth.tex2D(x+1, y-1) - d) - (d - depth.tex2D(x-1,y+1)));
			const float g = max(g1,max(g2,(max(g3,g4))));

			// Calculate support window area
			//const uchar4 sup = support.tex2D((int)x, (int)y);
			const uchar4 sup = getScaledTex2D(x, y, support, depth);
			const float supx = min(sup.x,sup.y);
			const float supy = min(sup.z,sup.w);
			const float area = supx * supx * supy;

			float grad_weight = min(1.0f, g / threshold);
			float area_weight = min(1.0f, area / params.areaMax);

			if (grad_weight * (1.0f - area_weight) > params.disconThresh) mask.isDiscontinuity(true);
			if (grad_weight * (area_weight) > params.noiseThresh) mask.isNoise(true);

			// High change of gradient and large area means noise
			if (params.noise) weight *= (1.0f - square(grad_weight * area_weight));

			// Really high area means planar so unlikely to be reliable.
			//if (params.colour) weight *= (min(1.2f, (1.8f - area_weight)));
			if (params.colour) weight *= (min(1.0f, (1.0f - area_weight)));
		}

		mask_out(x,y) = (int)mask;
		weight_out(x,y) = short(min(1.0f, weight) * 32767.0f);
	}
}

void ftl::cuda::pixel_weighting(
		ftl::cuda::TextureObject<short> &weight_out,
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask_out,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &camera,
		const cv::Size size, const ftl::cuda::PixelWeightingParameters &params,
		cudaStream_t stream) {

	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	pixel_weight_kernel<<<gridSize, blockSize, 0, stream>>>(weight_out, mask_out, ftl::cuda::TextureObject<half4>(), support, depth, camera, size, params);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

void ftl::cuda::pixel_weighting(
		ftl::cuda::TextureObject<short> &weight_out,
		ftl::cuda::TextureObject<ftl::cuda::Mask::type> &mask_out,
		ftl::cuda::TextureObject<half4> &normals_out,
		ftl::cuda::TextureObject<uchar4> &support,
		ftl::cuda::TextureObject<float> &depth,
		const ftl::rgbd::Camera &camera,
		const cv::Size size, const ftl::cuda::PixelWeightingParameters &params,
		cudaStream_t stream) {

	const dim3 gridSize((size.width + T_PER_BLOCK - 1)/T_PER_BLOCK, (size.height + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	pixel_weight_kernel<<<gridSize, blockSize, 0, stream>>>(weight_out, mask_out, normals_out, support, depth, camera, size, params);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}

// =============================================================================

__global__ void cull_weight_kernel(ftl::cuda::TextureObject<short> weights, ftl::cuda::TextureObject<float> depth, float thresh) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		if (float(weights.tex2D(x,y)) / 32767.0f < thresh) {
			depth(x,y) = 0.0f;
		}
	}
}

void ftl::cuda::cull_weight(ftl::cuda::TextureObject<short> &weights, ftl::cuda::TextureObject<float> &depth, float thresh, cudaStream_t stream) {
	const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	cull_weight_kernel<<<gridSize, blockSize, 0, stream>>>(weights, depth, thresh);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

// =============================================================================

template <int RADIUS, bool INVERT>
__global__ void degrade_mask_kernel(ftl::cuda::TextureObject<uint8_t> mask,
		ftl::cuda::TextureObject<short> weights, uint8_t id) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < weights.width()-RADIUS && y < weights.height()-RADIUS) {
		float distance = 100.0f;

		#pragma unroll
		for (int v=-RADIUS; v<=RADIUS; ++v) {
		#pragma unroll
		for (int u=-RADIUS; u<=RADIUS; ++u) {
			Mask m(mask.tex2D((int)x+u,(int)y+v));
			if (m.is(id)) distance = min(distance, sqrt(float(v*v + u*u)));
		}
		}

		if (distance < 99.0f) {
			const float w = float(weights(x,y));
			if (INVERT) weights(x,y) = short(w + (32767.0f-w) * (ftl::cuda::weighting(distance, float(RADIUS+1))));
			else weights(x,y) = short(w * (1.0f - ftl::cuda::weighting(distance, float(RADIUS+1))));
		}
	}
}

void ftl::cuda::degrade_mask(ftl::cuda::TextureObject<uint8_t> &mask, ftl::cuda::TextureObject<short> &weights, uint8_t id, unsigned int radius, bool invert, cudaStream_t stream) {
	const dim3 gridSize((weights.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (weights.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (invert) {
		switch (radius) {
		case 0	: degrade_mask_kernel<0,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 1	: degrade_mask_kernel<1,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 2	: degrade_mask_kernel<2,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 3	: degrade_mask_kernel<3,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 4	: degrade_mask_kernel<4,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 5	: degrade_mask_kernel<5,true><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		default: break;
		}
	} else {
		switch (radius) {
		case 0	: degrade_mask_kernel<0,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 1	: degrade_mask_kernel<1,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 2	: degrade_mask_kernel<2,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 3	: degrade_mask_kernel<3,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 4	: degrade_mask_kernel<4,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		case 5	: degrade_mask_kernel<5,false><<<gridSize, blockSize, 0, stream>>>(mask, weights, id); break;
		default: break;
		}
	}
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
