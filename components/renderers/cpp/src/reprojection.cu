#include <ftl/render/render_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/weighting.hpp>
#include <ftl/cuda/makers.hpp>

#define T_PER_BLOCK 8
#define ACCUM_DIAMETER 8

using ftl::cuda::TextureObject;
using ftl::render::Parameters;
using ftl::rgbd::Camera;
using ftl::render::ViewPortMode;
using ftl::render::AccumulationFunction;
using ftl::rgbd::Projection;

/*template <typename T>
__device__ inline T generateInput(const T &in, const SplatParams &params, const float4 &worldPos) {
	return in;
}

template <>
__device__ inline uchar4 generateInput(const uchar4 &in, const SplatParams &params, const float4 &worldPos) {
	return (params.m_flags & ftl::render::kShowDisconMask && worldPos.w < 0.0f) ?
		make_uchar4(0,0,255,255) :  // Show discontinuity mask in red
		in;
}*/

template <typename A, typename B>
__device__ inline B weightInput(const A &in, float weight) {
	return in * weight;
}

template <>
__device__ inline float4 weightInput(const uchar4 &in, float weight) {
	return make_float4(
		(float)in.x * weight,
		(float)in.y * weight,
		(float)in.z * weight,
		(float)in.w * weight);
}

template <typename T>
__device__ inline float colourDifference(const T &a, const T &b);

template <>
__device__ inline float colourDifference<float4>(const float4 &a, const float4 &b) {
	return max(fabsf(a.x-b.x), max(fabsf(a.y-b.y), fabsf(a.z-b.z)));
}

template <>
__device__ inline float colourDifference<float>(const float &a, const float &b) {
	return fabs(a-b);
}

template <AccumulationFunction F, typename T>
__device__ inline void accumulateOutput(TextureObject<T> &out, TextureObject<int> &contrib, const uint2 &pos, const T &in, float w) {
	// Just weighted average everything
	if (F == AccumulationFunction::Simple) {
		const T old = out.tex2D(pos.x, pos.y);
		const int c = contrib.tex2D(pos.x,pos.y);
		out(pos.x, pos.y) = old + in*w;
		contrib(pos.x, pos.y) = int(w * float(0xFFFF)) + (c & 0xFFFFFF);
	} else {
		int c = contrib.tex2D(pos.x,pos.y);
		float weight_sum = float(c & 0xFFFFFF) / float(0xFFFF);
		float count = c >> 24;
		const T old = out.tex2D(pos.x, pos.y);

		// Really close weights are weighted averaged together
		// but substantially stronger weights do a straight replacement
		if (F == AccumulationFunction::CloseWeights) {
			if (count == 0 || w*0.95f > weight_sum/count) {
				out(pos.x, pos.y) = in*w;
				contrib(pos.x, pos.y) = (1 << 24) + int(w * float(0xFFFF));
			} else {
				out(pos.x, pos.y) = old + in*w;
				contrib(pos.x, pos.y) = (int(count+1.0f) << 24) + int(w * float(0xFFFF)) + (c & 0xFFFFFF);
			}
		// The winner takes all in determining colour
		} else if (F == AccumulationFunction::BestWeight) {
			if (count == 0 || w > weight_sum/count) {
				out(pos.x, pos.y) = in*w;
				contrib(pos.x, pos.y) = (1 << 24) + int(w * float(0xFFFF));
			}
		// If colours are close then weighted average, otherwise discard the
		// lowest weighted colours
		} else if (F == AccumulationFunction::ColourDiscard) {
			if (colourDifference(old / weight_sum, in) > 10.0f) {
				if (count == 0 || w > weight_sum/count) {
					out(pos.x, pos.y) = in*w;
					contrib(pos.x, pos.y) = (1 << 24) + int(w * float(0xFFFF));
				} else {
					//out(pos.x, pos.y) = old + in*w;
					//contrib(pos.x, pos.y) = (int(count+1.0f) << 24) + int(w * float(0xFFFF)) + (c & 0xFFFFFF);
				}
			} else {
				out(pos.x, pos.y) = old + in*w;
				contrib(pos.x, pos.y) = (int(count+1.0f) << 24) + int(w * float(0xFFFF)) + (c & 0xFFFFFF);
			}
		} else if (F == AccumulationFunction::ColourDiscardSmooth) {
			//const float cdiff = 1.0f - min(1.0f, colourDifference(old / weight_sum, in) / 50.0f);
			// TODO: Determine colour smoothing param from magnitude of weighting difference
			// Or at least consider the magnitude of weight difference some how.
			const float cdiff = ftl::cuda::weighting(colourDifference(old / weight_sum, in), 255.0f);
			const float alpha = (w > weight_sum/count) ? cdiff : 1.0f;
			const float beta = (count == 0 || w > weight_sum/count) ? 1.0f : cdiff;

			out(pos.x, pos.y) = old*alpha + in*w*beta;
			contrib(pos.x, pos.y) = (int(count+1.0f) << 24) + int(w*beta * float(0xFFFF)) + int(weight_sum*alpha * float(0xFFFF));
		}
	}
} 

template <ViewPortMode VPMODE>
__device__ inline float2 convertScreen(const Parameters &params, int x, int y) {
	return make_float2(x,y);
}

/*template <>
__device__ inline float2 convertScreen<ViewPortMode::Warping>(const Parameters &params, int x, int y) {
	const float coeff = 1.0f / (params.viewport.warpMatrix.entries[6] * x + params.viewport.warpMatrix.entries[7] * y + params.viewport.warpMatrix.entries[8]);
	const float xcoo = coeff * (params.viewport.warpMatrix.entries[0] * x + params.viewport.warpMatrix.entries[1] * y + params.viewport.warpMatrix.entries[2]);
	const float ycoo = coeff * (params.viewport.warpMatrix.entries[3] * x + params.viewport.warpMatrix.entries[4] * y + params.viewport.warpMatrix.entries[5]);
	float2 pt = params.viewport.reverseMap(params.camera, make_float2(xcoo,ycoo));
	return pt;
}*/

template <>
__device__ inline float2 convertScreen<ViewPortMode::Stretch>(const Parameters &params, int x, int y) {
	return params.viewport.reverseMap(params.camera, make_float2(x,y));
}

template <typename A>
__device__ inline auto getInput(TextureObject<A> &in, const float3 &screen, float width, float height) {
	const float inSX = float(in.width()) / width;
	const float inSY = float(in.height()) / height;
	return in.tex2D(screen.x*inSX, screen.y*inSY); 
}

__device__ float weightByNormal(TextureObject<half4> &normals, int x, int y, const float3x3 &transformR, const float3 &screenPos, const ftl::rgbd::Camera &camera) {
	// Calculate the dot product of surface normal and camera ray
	const float3 n = transformR * make_float3(normals.tex2D(x, y));
	float3 ray = camera.screenToCam(screenPos.x, screenPos.y, 1.0f);
	ray = ray / length(ray);

	// Allow slightly beyond 90 degrees due to normal estimation errors
	const float dotproduct = (max(dot(ray,n),-0.1f)+0.1) / 1.1f;
	return dotproduct;
}

__device__ float depthMatching(const Parameters &params, float d1, float d2) {
	// TODO: Consider a slightly different pixel size check
	const float threshold = (params.depthCoef / ((params.depthCoef / d2) - 1.0f)) - d2;
	return (fabs(d1 - d2) <= threshold) ? 1.0f : 0.0f;
}

/*
 * Full reprojection with normals and depth
 */
 template <typename A, typename B, AccumulationFunction ACCUM, Projection PROJECT>
__global__ void reprojection_kernel(
        TextureObject<A> in,				// Attribute input
        TextureObject<float> depth_src,
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<short> weights,
		TextureObject<half4> normals,
		TextureObject<B> out,			// Accumulated output
		TextureObject<int> contrib,
		Parameters params,
		Camera camera, float4x4 transform, float3x3 transformR) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = depth_in.tex2D((int)x, (int)y);
	if (d > params.camera.minDepth && d < params.camera.maxDepth) {
		//const float2 rpt = convertScreen<VPMODE>(params, x, y);
		//const float3 camPos = transform * params.camera.screenToCam(rpt.x, rpt.y, d);
		const float3 camPos = transform * params.camera.unproject<PROJECT>(make_float3(x, y, d));
		if (camPos.z > camera.minDepth && camPos.z < camera.maxDepth) {
			const float3 screenPos = camera.project<Projection::PERSPECTIVE>(camPos);

			// Not on screen so stop now...
			if (screenPos.x < depth_src.width() && screenPos.y < depth_src.height()) {
				const float d2 = depth_src.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f));

				const auto input = getInput(in, screenPos, depth_src.width(), depth_src.height()); 

				// Boolean match (0 or 1 weight). 1.0 if depths are sufficiently close
				float weight = depthMatching(params, camPos.z, d2);

				if (params.m_flags & ftl::render::kUseWeightsChannel)
					weight *= float(weights.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f))) / 32767.0f;

				// TODO: Weight by distance to discontinuity? Perhaps as an alternative to
				// removing a discontinuity. This would also gradually blend colours from
				// multiple cameras that otherwise might jump when one cameras input is lost
				// at an invalid patch.

				/* Buehler C. et al. 2001. Unstructured Lumigraph Rendering. */
				/* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
				// This is the simple naive colour weighting. It might be good
				// enough for our purposes if the alignment step prevents ghosting
				// TODO: Use depth and perhaps the neighbourhood consistency in:
				//     Kuster C. et al. 2011. FreeCam: A hybrid camera system for interactive free-viewpoint video
				if (params.m_flags & ftl::render::kNormalWeightColours)
					weight *= weightByNormal(normals, x, y, transformR, screenPos, camera);

				const B output = make<B>(input);  // * weight; //weightInput(input, weight);

				if (weight > 0.0f) {
					accumulateOutput<ACCUM,B>(out, contrib, make_uint2(x,y), output, weight);
				}
			}
		}
	}
}

/*
 * Full reprojection without normals
 */
 template <typename A, typename B, AccumulationFunction ACCUM, Projection PROJECT>
__global__ void reprojection_kernel(
        TextureObject<A> in,				// Attribute input
        TextureObject<float> depth_src,
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<short> weights,
		TextureObject<B> out,			// Accumulated output
		TextureObject<int> contrib,
		Parameters params,
		Camera camera, float4x4 transform, float3x3 transformR) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = depth_in.tex2D((int)x, (int)y);
	if (d > params.camera.minDepth && d < params.camera.maxDepth) {
		//const float2 rpt = convertScreen<VPMODE>(params, x, y);
		const float3 camPos = transform * params.camera.unproject<PROJECT>(make_float3(x, y, d));
		if (camPos.z > camera.minDepth && camPos.z < camera.maxDepth) {
			const float3 screenPos = camera.project<Projection::PERSPECTIVE>(camPos);

			// Not on screen so stop now...
			if (screenPos.x < depth_src.width() && screenPos.y < depth_src.height()) {
				const float d2 = depth_src.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f));
				const auto input = getInput(in, screenPos, depth_src.width(), depth_src.height()); 

				// Boolean match (0 or 1 weight). 1.0 if depths are sufficiently close
				float weight = depthMatching(params, camPos.z, d2);
				if (params.m_flags & ftl::render::kUseWeightsChannel)
					weight *= float(weights.tex2D(int(screenPos.x+0.5f), int(screenPos.y+0.5f))) / 32767.0f;

				const B output = make<B>(input);  // * weight; //weightInput(input, weight);

				if (weight > 0.0f) {
					accumulateOutput<ACCUM,B>(out, contrib, make_uint2(x,y), output, weight);
				}
			}
		}
	}
}


template <typename A, typename B>
void ftl::cuda::reproject(
        TextureObject<A> &in,
        TextureObject<float> &depth_src,       // Original 3D points
		TextureObject<float> &depth_in,        // Virtual depth map
		TextureObject<short> &weights,
		TextureObject<half4> *normals,
		TextureObject<B> &out,   // Accumulated output
		TextureObject<int> &contrib,
		const Parameters &params,
		const Camera &camera, const float4x4 &transform, const float3x3 &transformR,
		cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (normals) {
		if (params.accumulationMode == AccumulationFunction::CloseWeights) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::CloseWeights><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::BestWeight) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::BestWeight><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::Simple) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::Simple><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::ColourDiscard) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::ColourDiscard><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::ColourDiscardSmooth) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::ColourDiscardSmooth><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, *normals, out, contrib, params, camera, transform, transformR); break;
			}
		}
	} else {
		if (params.accumulationMode == AccumulationFunction::CloseWeights) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::CloseWeights, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::CloseWeights><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::BestWeight) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::BestWeight, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::BestWeight><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::Simple) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::Simple, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::Simple><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::ColourDiscard) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::ColourDiscard, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::ColourDiscard><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			}
		} else if (params.accumulationMode == AccumulationFunction::ColourDiscardSmooth) {
			switch (params.projection) {
			case Projection::PERSPECTIVE: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::PERSPECTIVE><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::ORTHOGRAPHIC: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::ORTHOGRAPHIC><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			case Projection::EQUIRECTANGULAR: reprojection_kernel<A,B,AccumulationFunction::ColourDiscardSmooth, Projection::EQUIRECTANGULAR><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			//case ViewPortMode::Stretch: reprojection_kernel<A,B,ViewPortMode::Stretch,AccumulationFunction::ColourDiscardSmooth><<<gridSize, blockSize, 0, stream>>>(in, depth_src, depth_in, weights, out, contrib, params, camera, transform, transformR); break;
			}
		}
	}
	cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::reproject(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
	ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<short> &weights,
	ftl::cuda::TextureObject<half4> *normals,
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<int> &contrib,
	const ftl::render::Parameters &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &transform, const float3x3 &transformR,
	cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<short> &weights,
		ftl::cuda::TextureObject<half4> *normals,
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR,
		cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_src,		// Original 3D points
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<short> &weights,
		ftl::cuda::TextureObject<half4> *normals,
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &transform, const float3x3 &transformR,
		cudaStream_t stream);


//==============================================================================
//  Without normals or depth
//==============================================================================

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
 template <typename A, typename B>
__global__ void reprojection_kernel(
        TextureObject<A> in,				// Attribute input
		TextureObject<float> depth_in,        // Virtual depth map
		TextureObject<B> out,			// Accumulated output
		TextureObject<int> contrib,
		Parameters params,
		Camera camera, float4x4 poseInv) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d = depth_in.tex2D((int)x, (int)y);
	if (d > params.camera.minDepth && d < params.camera.maxDepth) {
		//const float3 camPos = poseInv * params.camera.screenToCam(x, y, d);
		const float3 camPos = poseInv * params.camera.unproject<Projection::PERSPECTIVE>(make_float3(x, y, d));
		const float3 screenPos = camera.project<Projection::PERSPECTIVE>(camPos);

		if (screenPos.x < in.width() && screenPos.y < in.height()) {
			const auto input = in.tex2D(screenPos.x, screenPos.y);
			float weight = depthMatching(params, camPos.z, camera.maxDepth);
			const B weighted = make<B>(input) * weight;

			if (weight > 0.0f) {
				accumulateOutput<AccumulationFunction::Simple>(out, contrib, make_uint2(x,y), weighted, weight);
			}
		}
	}
}


template <typename A, typename B>
void ftl::cuda::reproject(
        TextureObject<A> &in,
		TextureObject<float> &depth_in,        // Virtual depth map
		TextureObject<B> &out,   // Accumulated output
		TextureObject<int> &contrib,
		const Parameters &params,
		const Camera &camera, const float4x4 &poseInv, cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    reprojection_kernel<<<gridSize, blockSize, 0, stream>>>(
        in,
		depth_in,
		out,
		contrib,
		params,
		camera,
		poseInv
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::reproject(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<int> &contrib,
	const ftl::render::Parameters &params,
	const ftl::rgbd::Camera &camera,
	const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);

template void ftl::cuda::reproject(
		ftl::cuda::TextureObject<float4> &in,	// Original colour image
		ftl::cuda::TextureObject<float> &depth_in,		// Virtual depth map
		ftl::cuda::TextureObject<float4> &out,	// Accumulated output
		ftl::cuda::TextureObject<int> &contrib,
		const ftl::render::Parameters &params,
		const ftl::rgbd::Camera &camera,
		const float4x4 &poseInv, cudaStream_t stream);


// ===== Equirectangular Reprojection ==========================================

__device__ inline float2 equirect_reprojection(int x_img, int y_img, double f, const float3x3 &rot, int w1, int h1, const ftl::rgbd::Camera &cam) {
	float3 ray3d = cam.screenToCam(x_img, y_img, 1.0f);
	ray3d /= length(ray3d);
	ray3d = rot * ray3d;

    //inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    float theta = atan2(ray3d.y,sqrt(ray3d.x*ray3d.x+ray3d.z*ray3d.z));
	float phi = atan2(ray3d.x, ray3d.z);
	
	const float pi = 3.14f;

    //get 2D point on equirectangular map
    float x_sphere = (((phi*w1)/pi+w1)/2); 
    float y_sphere = (theta+ pi/2)*h1/pi;

    return make_float2(x_sphere,y_sphere);
};

__global__ void equirectangular_kernel(
		TextureObject<uchar4> image_in,
		TextureObject<uchar4> image_out,
		Camera camera, float3x3 pose) {
		
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < image_out.width() && y < image_out.height()) {
		const float2 p = equirect_reprojection(x,y, camera.fx, pose, image_in.width(), image_in.height(), camera);
		const float4 colour = image_in.tex2D(p.x, p.y);
		image_out(x,y) = make_uchar4(colour.x, colour.y, colour.z, 0);
	}
}

void ftl::cuda::equirectangular_reproject(
		ftl::cuda::TextureObject<uchar4> &image_in,
		ftl::cuda::TextureObject<uchar4> &image_out,
		const ftl::rgbd::Camera &camera, const float3x3 &pose, cudaStream_t stream) {

	const dim3 gridSize((image_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (image_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	equirectangular_kernel<<<gridSize, blockSize, 0, stream>>>(image_in, image_out, camera, pose);
	cudaSafeCall( cudaGetLastError() );
}

// ==== Correct for bad colours ================================================

__device__ inline uchar4 make_uchar4(const float4 v) {
	return make_uchar4(v.x,v.y,v.z,v.w);
}

template <int RADIUS>
__global__ void fix_colour_kernel(
		TextureObject<float> depth,
		TextureObject<uchar4> out,
		TextureObject<int> contribs,
		uchar4 bad_colour,
		ftl::rgbd::Camera cam) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= RADIUS && y >= RADIUS && x < out.width()-RADIUS && y < out.height()-RADIUS) {
		const float contrib = contribs.tex2D((int)x,(int)y);
		const float d = depth.tex2D(x,y);

		if (contrib == 0 && d > cam.minDepth && d < cam.maxDepth) {
			float4 sumcol = make_float4(0.0f);
			float count = 0.0f;

			for (int v=-RADIUS; v<=RADIUS; ++v) {
				for (int u=-RADIUS; u<=RADIUS; ++u) {
					const int contrib = contribs.tex2D((int)x+u,(int)y+v);
					const float4 c = make_float4(out(int(x)+u,int(y)+v));
					if (contrib > 0) {
						sumcol += c;
						count += 1.0f;
					}
				}
			}

			out(x,y) = (count > 0.0f) ? make_uchar4(sumcol / count) : bad_colour;
		}
	}
}

void ftl::cuda::fix_bad_colour(
		TextureObject<float> &depth,
		TextureObject<uchar4> &out,
		TextureObject<int> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	fix_colour_kernel<1><<<gridSize, blockSize, 0, stream>>>(depth, out, contribs, bad_colour, cam);
	cudaSafeCall( cudaGetLastError() );
}

// ===== Show bad colour normalise =============================================

__global__ void show_missing_colour_kernel(
		TextureObject<float> depth,
		TextureObject<uchar4> out,
		TextureObject<int> contribs,
		uchar4 bad_colour,
		ftl::rgbd::Camera cam) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < out.width() && y < out.height()) {
		const int contrib = contribs.tex2D((int)x,(int)y);
		const float d = depth.tex2D(x,y);

		if (contrib == 0 && d > cam.minDepth && d < cam.maxDepth) {
			out(x,y) = bad_colour;
		}
	}
}

void ftl::cuda::show_missing_colour(
		TextureObject<float> &depth,
		TextureObject<uchar4> &out,
		TextureObject<int> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_missing_colour_kernel<<<gridSize, blockSize, 0, stream>>>(depth, out, contribs, bad_colour, cam);
	cudaSafeCall( cudaGetLastError() );
}

// ===== Show colour weights ===================================================

__global__ void show_colour_weights_kernel(
		TextureObject<uchar4> out,
		TextureObject<int> contribs,
		uchar4 bad_colour) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < out.width() && y < out.height()) {
		const int contrib = contribs.tex2D((int)x,(int)y);

		if (contrib > 0) {
			float w = float(contrib & 0xFFFFFF) / float(0xFFFF) / float(contrib >> 24);
			out(x,y) = make_uchar4(float(bad_colour.x) * w, float(bad_colour.y) * w, float(bad_colour.z) * w, 0.0f);
		}
	}
}

void ftl::cuda::show_colour_weights(
		TextureObject<uchar4> &out,
		TextureObject<int> &contribs,
		uchar4 bad_colour,
		cudaStream_t stream) {
	const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	show_colour_weights_kernel<<<gridSize, blockSize, 0, stream>>>(out, contribs, bad_colour);
	cudaSafeCall( cudaGetLastError() );
}
