#include "mvmls_cuda.hpp"
#include <ftl/cuda/weighting.hpp>
#include <ftl/operators/mask_cuda.hpp>
#include <ftl/cuda/warp.hpp>

using ftl::cuda::TextureObject;
using ftl::rgbd::Camera;
using ftl::cuda::Mask;
using ftl::cuda::MvMLSParams;

#define T_PER_BLOCK 8
#define WARP_SIZE 32

#include "correspondence_common.hpp"


// ==== MultiViewMLS Aggregate =================================================

__device__ inline short3 getScreenPos(int x, int y, float d, const Camera &cam1, const Camera &cam2, const float4x4 &transform) {
    const float3 campos = transform * cam1.screenToCam(x,y,d);
    const int2 screen = cam2.camToScreen<int2>(campos);
    return make_short3(screen.x, screen.y, campos.z);
}

__device__ inline short2 packScreen(int x, int y, int id) {
    return make_short2((id << 12) + x, y);
}

__device__ inline short2 packScreen(const short3 &p, int id) {
    return make_short2((id << 12) + p.x, p.y);
}

__device__ inline int supportSize(uchar4 support) {
    return (support.x+support.y) * (support.z+support.w);
}

__device__ inline short2 choosePoint(uchar4 sup1, uchar4 sup2, float dot1, float dot2, short2 screen1, short2 screen2) {
    //return (float(supportSize(sup2))*dot1 > float(supportSize(sup1))*dot2) ? screen2 : screen1;
    return (dot1 > dot2) ? screen2 : screen1;
}

__device__ inline int unpackCameraID(short2 p) {
    return p.x >> 12;
}

/**
 * Identify which source has the best support region for a given pixel.
 */
__global__ void best_sources_kernel(
        TextureObject<half4> normals1,
        TextureObject<half4> normals2,
        TextureObject<uchar4> support1,
        TextureObject<uchar4> support2,
        TextureObject<float> depth1,
        TextureObject<float> depth2,
        TextureObject<short2> screen,
        float4x4 transform,
        //float3x3 transformR,
        ftl::rgbd::Camera cam1,
        ftl::rgbd::Camera cam2,
        int id1,
        int id2) {

    const int x = (blockIdx.x*blockDim.x + threadIdx.x);
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= 0 && y >= 0 && x < screen.width() && y < screen.height()) {
        const float d1 = depth1.tex2D(x,y);

        const short3 scr2 = getScreenPos(x, y, d1, cam1, cam2, transform);
        short2 bestPoint = packScreen(x,y,0);

        if (scr2.x >= 0 && scr2.y >= 0 && scr2.x < cam2.width && scr2.y < cam2.height) {
            uchar4 sup1 = support1.tex2D(x,y);
            uchar4 sup2 = support2.tex2D(scr2.x,scr2.y);
            const float d2 = depth2.tex2D(scr2.x,scr2.y);
            float3 n1 = transform.getFloat3x3() * make_float3(normals1.tex2D(x,y));
            float3 n2 = make_float3(normals2.tex2D(scr2.x,scr2.y));

            float3 camray = cam2.screenToCam(scr2.x,scr2.y,1.0f);
            camray /= length(camray);
            const float dot1 = dot(camray, n1);
            const float dot2 = dot(camray, n2);

            bestPoint = (fabs(scr2.z - d2) < 0.04f) ? choosePoint(sup1, sup2, dot1, dot2, packScreen(x,y,id1), packScreen(scr2,id2)) : packScreen(x,y,6);
            //bestPoint = choosePoint(sup1, sup2, dot1, dot2, packScreen(x,y,id1), packScreen(scr2,id2));
			//bestPoint = (d1 < d2) ? packScreen(x,y,id1) : packScreen(x,y,id2);
			
			bestPoint = (fabs(scr2.z - d2) < 0.04f) ? packScreen(scr2,id2) : packScreen(scr2,id1);
        }

        screen(x,y) = bestPoint;

        /*if (s.x >= 0 && s.y >= 0) {
            auto norm1 = make_float3(n1.tex2D(x,y));
            const auto norm2 = make_float3(n2.tex2D(s.x,s.y));
            //n2(s.x,s.y) = norm1;

            float3 cent1 = make_float3(c1.tex2D(x,y));
            const auto cent2 = make_float3(c2.tex2D(s.x,s.y));

            if (cent2.x+cent2.y+cent2.z > 0.0f && norm2.x+norm2.y+norm2.z > 0.0f) {
                norm1 += poseInv1.getFloat3x3() * (pose2.getFloat3x3() * norm2);
                n1(x,y) = make_float4(norm1, 0.0f);
				cent1 +=  poseInv1 * (pose2 * cent2);  // FIXME: Transform between camera spaces
				cent1 /= 2.0f;
                c1(x,y) = make_float4(cent1, 0.0f);
                //c2(s.x,s.y) = cent1;

				//contribs1(x,y) = contribs1.tex2D(x,y) + 1.0f;
            }
           // contribs2(s.x,s.y) = contribs2.tex2D(s.x,s.y) + 1.0f;
        }*/
    }
}

void ftl::cuda::best_sources(
        ftl::cuda::TextureObject<half4> &normals1,
        ftl::cuda::TextureObject<half4> &normals2,
        ftl::cuda::TextureObject<uchar4> &support1,
        ftl::cuda::TextureObject<uchar4> &support2,
        ftl::cuda::TextureObject<float> &depth1,
        ftl::cuda::TextureObject<float> &depth2,
        ftl::cuda::TextureObject<short2> &screen,
        const float4x4 &transform,
        const ftl::rgbd::Camera &cam1,
        const ftl::rgbd::Camera &cam2,
        int id1,
        int id2,
        cudaStream_t stream) {

    const dim3 gridSize((screen.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (screen.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    best_sources_kernel<<<gridSize, blockSize, 0, stream>>>(normals1, normals2, support1, support2, depth1, depth2, screen, transform, cam1, cam2, id1, id2);
    cudaSafeCall( cudaGetLastError() );
}

/**
 * Identify which source has the best support region for a given pixel.
 */
 __global__ void aggregate_sources_kernel(
		TextureObject<half4> n1,
		TextureObject<half4> n2,
		TextureObject<float4> c1,
		TextureObject<float4> c2,
		TextureObject<float> depth1,
		//TextureObject<float> depth2,
		//TextureObject<short2> screen,
		float4x4 transform,
		//float3x3 transformR,
		ftl::rgbd::Camera cam1,
		ftl::rgbd::Camera cam2) {

	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x >= 0 && y >= 0 && x < n1.width() && y < n1.height()) {
		const float d1 = depth1.tex2D(x,y);

		if (d1 > cam1.minDepth && d1 < cam1.maxDepth) {
			//const short3 s = getScreenPos(x, y, d1, cam1, cam2, transform);
			const float3 camPos = transform * cam1.screenToCam(x, y, d1);
			const int2 s = cam2.camToScreen<int2>(camPos);

			if (s.x >= 0 && s.y >= 0 && s.x < n2.width() && s.y < n2.height()) {
				auto norm1 = make_float3(n1.tex2D(x,y));
				const auto norm2 = make_float3(n2.tex2D(s.x,s.y));
				//n2(s.x,s.y) = norm1;

				float3 cent1 = make_float3(c1.tex2D(x,y));
				const auto cent2 = transform.getInverse() * make_float3(c2.tex2D(s.x,s.y));

				//printf("MERGING %f\n", length(cent2-cent1));

				if (cent2.x+cent2.y+cent2.z > 0.0f && norm2.x+norm2.y+norm2.z > 0.0f && length(cent2-cent1) < 0.04f) {
					norm1 += norm2;
					norm1 /= 2.0f;
					n1(x,y) = make_half4(norm1, 0.0f);
					cent1 += cent2;
					cent1 /= 2.0f;
					c1(x,y) = make_float4(cent1, 0.0f);
					//c2(s.x,s.y) = cent1;

					//contribs1(x,y) = contribs1.tex2D(x,y) + 1.0f;
				}
			// contribs2(s.x,s.y) = contribs2.tex2D(s.x,s.y) + 1.0f;
			}
		}
	}
}

void ftl::cuda::aggregate_sources(
		ftl::cuda::TextureObject<half4> &n1,
		ftl::cuda::TextureObject<half4> &n2,
		ftl::cuda::TextureObject<float4> &c1,
		ftl::cuda::TextureObject<float4> &c2,
		ftl::cuda::TextureObject<float> &depth1,
		//ftl::cuda::TextureObject<float> &depth2,
		//ftl::cuda::TextureObject<short2> &screen,
		const float4x4 &transform,
		const ftl::rgbd::Camera &cam1,
		const ftl::rgbd::Camera &cam2,
		cudaStream_t stream) {

	const dim3 gridSize((n1.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (n1.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	aggregate_sources_kernel<<<gridSize, blockSize, 0, stream>>>(n1, n2, c1, c2, depth1, transform, cam1, cam2);
	cudaSafeCall( cudaGetLastError() );
}

__device__ static uchar4 HSVtoRGB(int H, float S, float V) {
	const float C = S * V;
	const float X = C * (1 - fabs(fmodf(H / 60.0f, 2) - 1));
	const float m = V - C;
	float Rs, Gs, Bs;

	if(H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;	
	}
	else if(H >= 60 && H < 120) {	
		Rs = X;
		Gs = C;
		Bs = 0;	
	}
	else if(H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;	
	}
	else if(H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;	
	}
	else if(H >= 240 && H < 300) {
		Rs = X;
		Gs = 0;
		Bs = C;	
	}
	else {
		Rs = C;
		Gs = 0;
		Bs = X;	
	}

	return make_uchar4((Bs + m) * 255, (Gs + m) * 255, (Rs + m) * 255, 0);
}

/**
 * Render each pixel is a colour corresponding to the source camera with the
 * best support window.
 */
 __global__ void vis_best_sources_kernel(
        TextureObject<short2> screen,
        TextureObject<uchar4> colour,
        int myid,
        int count) {

    const int x = (blockIdx.x*blockDim.x + threadIdx.x);
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x >= 0 && y >= 0 && x < colour.width() && y < colour.height()) {
        short2 s = screen.tex2D(x,y);
        int id = unpackCameraID(s);

        uchar4 c = HSVtoRGB((360 / count) * id, 0.6f, 0.85f);
        if (myid != id) colour(x,y) = c;
        //colour(x,y) = c;
    }
}

void ftl::cuda::vis_best_sources(
        ftl::cuda::TextureObject<short2> &screen,
        ftl::cuda::TextureObject<uchar4> &colour,
        int myid,
        int count,
        cudaStream_t stream) {

    const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    vis_best_sources_kernel<<<gridSize, blockSize, 0, stream>>>(screen, colour, myid, count);
    cudaSafeCall( cudaGetLastError() );
}

/*void ftl::cuda::aggregate_sources(
        ftl::cuda::TextureObject<float4> &n1,
        ftl::cuda::TextureObject<float4> &n2,
        ftl::cuda::TextureObject<float4> &c1,
        ftl::cuda::TextureObject<float4> &c2,
        ftl::cuda::TextureObject<float> &contribs1,
        ftl::cuda::TextureObject<float> &contribs2,
		ftl::cuda::TextureObject<short2> &screen,
		const float4x4 &poseInv1,
		const float4x4 &pose2,
        cudaStream_t stream) {

    const dim3 gridSize((screen.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (screen.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    aggregate_sources_kernel<<<gridSize, blockSize, 0, stream>>>(n1, n2, c1, c2, contribs1, contribs2, screen, poseInv1, pose2);
    cudaSafeCall( cudaGetLastError() );
}*/

// ==== Normalise aggregations =================================================

__global__ void normalise_aggregations_kernel(
        TextureObject<half4> norms,
        TextureObject<float4> cents,
        TextureObject<float> contribs) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < norms.width() && y < norms.height()) {
        const float contrib = contribs.tex2D((int)x,(int)y);

        const auto a = make_float3(norms.tex2D((int)x,(int)y));
        const auto b = cents.tex2D(x,y);
        //const float4 normal = normals.tex2D((int)x,(int)y);

		//out(x,y) = (contrib == 0.0f) ? make<B>(a) : make<B>(a / contrib);

        if (contrib > 0.0f) {
            norms(x,y) = make_half4(a / (contrib+1.0f), 1.0f);
            cents(x,y) = b / (contrib+1.0f);
        }
    }
}

void ftl::cuda::normalise_aggregations(TextureObject<half4> &norms, TextureObject<float4> &cents, TextureObject<float> &contribs, cudaStream_t stream) {
    const dim3 gridSize((norms.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (norms.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    normalise_aggregations_kernel<<<gridSize, blockSize, 0, stream>>>(norms, cents, contribs);
    cudaSafeCall( cudaGetLastError() );
}
