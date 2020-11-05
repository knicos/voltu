#include "antialiasing_cuda.hpp"

#define T_PER_BLOCK 8

__device__ inline uchar4 toChar(const float4 rgba) {
    return make_uchar4(rgba.x*255.0f, rgba.y*255.0f, rgba.z*255.0f, 255);
}

__device__ void fxaa2(int x, int y, ftl::cuda::TextureObject<uchar4> &data) {
    uchar4 out_color;
    cudaTextureObject_t texRef = data.cudaTexture();

    const float FXAA_SPAN_MAX = 8.0f;
    const float FXAA_REDUCE_MUL = 1.0f/8.0f;
    const float FXAA_REDUCE_MIN = (1.0f/128.0f);

    float u = x + 0.5f;
    float v = y + 0.5f;

    float4 rgbNW = tex2D<float4>( texRef, u-1.0f,v-1.0f);
    float4 rgbNE = tex2D<float4>( texRef, u+1.0f,v-1.0f);
    float4 rgbSW = tex2D<float4>( texRef, u-1.0f,v+1.0f);
    float4 rgbSE = tex2D<float4>( texRef, u+1.0f,v+1.0f);
    float4 rgbM = tex2D<float4>( texRef, u,v);

    const float4 luma = make_float4(0.299f, 0.587f, 0.114f,0.0f);
    float lumaNW = dot(rgbNW, luma);
    float lumaNE = dot(rgbNE, luma);
    float lumaSW = dot(rgbSW, luma);
    float lumaSE = dot(rgbSE, luma);
    float lumaM = dot( rgbM, luma);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    float2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y = ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25f * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);

    float rcpDirMin = 1.0f/(min(abs(dir.x), abs(dir.y)) + dirReduce);


    float2 test = dir * rcpDirMin;
    dir = clamp(test,-FXAA_SPAN_MAX,FXAA_SPAN_MAX);

    float4 rgbA = (1.0f/2.0f) * (
                tex2D<float4>( texRef,u+ dir.x * (1.0f/3.0f - 0.5f),v+ dir.y * (1.0f/3.0f - 0.5f))+
                tex2D<float4>( texRef,u+ dir.x * (2.0f/3.0f - 0.5f),v+ dir.y * (2.0f/3.0f - 0.5f)));
    float4 rgbB = rgbA * (1.0f/2.0f) + (1.0f/4.0f) * (
                tex2D<float4>( texRef,u+ dir.x * (0.0f/3.0f - 0.5f),v+ dir.y * (0.0f/3.0f - 0.5f))+
                tex2D<float4>( texRef,u+ dir.x * (3.0f/3.0f - 0.5f),v+ dir.y * (3.0f/3.0f - 0.5f)));
    float lumaB = dot(rgbB, luma);

    if((lumaB < lumaMin) || (lumaB > lumaMax)){
        out_color=toChar(rgbA);
    } else {
        out_color=toChar(rgbB);
    }


    //surf2Dwrite<uchar4>(out_color, surfaceWrite, x*sizeof(uchar4), y);

	// FIXME: Should not output to same texture object.
    data(x,y) = out_color;
}

__global__ void filter_fxaa2(ftl::cuda::TextureObject<uchar4> data, ftl::cuda::TextureObject<float> depth, float threshold) {

    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= 0 && x < data.width() && y >= 0 && y < data.height())
    {
        // Do a depth discon test
        bool discon = false;
        float d = depth.tex2D(x,y);
        #pragma unroll
        for (int u=-1; u<=1; ++u) {
            #pragma unroll
            for (int v=-1; v<=1; ++v) {
                discon |= fabsf(d-depth.tex2D(x+u,y+v)) > threshold;
            }
        }

        if (discon) fxaa2(x, y, data);
    }
}

void ftl::cuda::fxaa(ftl::cuda::TextureObject<uchar4> &colour, ftl::cuda::TextureObject<float> &depth, float threshold, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    filter_fxaa2<<<gridSize, blockSize, 0, stream>>>(colour, depth, threshold);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

__global__ void filter_fxaa2(ftl::cuda::TextureObject<uchar4> data) {

    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x >= 0 && x < data.width() && y >= 0 && y < data.height())
    {
        fxaa2(x, y, data);
    }
}

void ftl::cuda::fxaa(ftl::cuda::TextureObject<uchar4> &colour, cudaStream_t stream) {
	const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    filter_fxaa2<<<gridSize, blockSize, 0, stream>>>(colour);
	cudaSafeCall( cudaGetLastError() );

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
