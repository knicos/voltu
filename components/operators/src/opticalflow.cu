#include "opticalflow_cuda.hpp"
#include <ftl/cuda/fixed.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::TextureObject;

__global__ void show_optflow_kernel(
        cv::cuda::PtrStepSz<short2> optflow,
        TextureObject<uchar4> colour, float scale) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < colour.width() && y < colour.height()) {
        short2 flow = optflow(y/4, x/4);  // 4 is granularity

        float fx = max(-1.0f, min(1.0f, fixed2float<5>(flow.x) / scale));
        float fy = max(-1.0f, min(1.0f, fixed2float<5>(flow.y) / scale));
        float f = sqrt(fx*fx+fy*fy);

        float4 c = colour.tex2D(float(x)+0.5f, float(y)+0.5f);
        c += make_float4(f*255.0f, 0.0f, f*255.0f, 0.0f);
        colour(x,y) = make_uchar4(min(255.0f, c.x), min(255.0f, c.y), min(255.0f, c.z), 0.0f);
    }
}

void ftl::cuda::show_optical_flow(const cv::cuda::GpuMat &optflow, const TextureObject<uchar4> &colour, float scale, cudaStream_t stream) {
    const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    show_optflow_kernel<<<gridSize, blockSize, 0, stream>>>(optflow, colour, scale);
    cudaSafeCall( cudaGetLastError() );
}

// ==== Flow difference ========================================================

__global__ void gen_disparity_kernel(
        cv::cuda::PtrStepSz<short2> optflow1,
        cv::cuda::PtrStepSz<short2> optflow2,
        TextureObject<short> disparity) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < disparity.width() && y < disparity.height()) {
        short2 flow1 = optflow1(y/4, x/4);  // 4 is granularity
        float disp = -fixed2float<5>(flow1.x);

        // Do a consistency check
        if (disp > 0.0f && x-disp-0.5f > 0) { //< colour.width()) {
            short2 flow2 = optflow2(y/4, (x-round(disp))/4);  // 4 is granularity
            if (fabsf(disp - fixed2float<5>(flow2.x)) > 1.0f) disp = 0.0f;
        }

        disparity(x,y) = float2fixed<4>(disp);
    }
}

void ftl::cuda::disparity_from_flow(const cv::cuda::GpuMat &optflow1, const cv::cuda::GpuMat &optflow2, const TextureObject<short> &disparity, cudaStream_t stream) {
    const dim3 gridSize((disparity.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (disparity.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    gen_disparity_kernel<<<gridSize, blockSize, 0, stream>>>(optflow1, optflow2, disparity);
    cudaSafeCall( cudaGetLastError() );
}

__global__ void show_optflow_diff_kernel(
        cv::cuda::PtrStepSz<short2> optflow1,
        cv::cuda::PtrStepSz<short2> optflow2,
        TextureObject<uchar4> colour, float scale,
        ftl::rgbd::Camera cam) {

    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < colour.width() && y < colour.height()) {
        short2 flow1 = optflow1(y/4, x/4);  // 4 is granularity
        float disp = fixed2float<5>(flow1.x);

        if (disp > 0.0f && x-disp-0.5f > 0) { //< colour.width()) {
            short2 flow2 = optflow2(y/4, (x-round(disp))/4);  // 4 is granularity

            float dx = (fixed2float<5>(flow1.x) + fixed2float<5>(flow2.x)) / disp;
            float fR = max(0.0f, min(1.0f, dx / scale));
            float fB = -max(-1.0f, min(0.0f, dx / scale));
            float fG = (fR == 1.0f || fB == 1.0f) ? 0.0f : 1.0f;

            float4 c = colour.tex2D(float(x)+0.5f, float(y)+0.5f);
            c += make_float4(fG*fB*255.0f, (1.0f-fG)*255.0f, fG*fR*255.0f, 0.0f);
            colour(x,y) = make_uchar4(min(255.0f, c.x), min(255.0f, c.y), min(255.0f, c.z), 0.0f);
        }
    }
}

void ftl::cuda::show_optical_flow_diff(const cv::cuda::GpuMat &optflow1, const cv::cuda::GpuMat &optflow2, const TextureObject<uchar4> &colour, float scale, const ftl::rgbd::Camera &cam, cudaStream_t stream) {
    const dim3 gridSize((colour.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    show_optflow_diff_kernel<<<gridSize, blockSize, 0, stream>>>(optflow1, optflow2, colour, scale, cam);
    cudaSafeCall( cudaGetLastError() );
}
