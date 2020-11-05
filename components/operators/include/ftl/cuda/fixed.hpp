#ifndef _FTL_CUDA_FIXED_HPP_
#define _FTL_CUDA_FIXED_HPP_

template <int FRAC>
__device__ inline float fixed2float(short v) {
    return float(v) / float(1 << FRAC);
}

template <int FRAC>
__device__ inline short float2fixed(float v) {
    return short(v * float(1 << FRAC));
}

#endif