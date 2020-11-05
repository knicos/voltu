#ifndef _CUDATL_FIXED_HPP_
#define _CUDATL_FIXED_HPP_

namespace cudatl
{

template <int FRAC>
__device__ inline float fixed2float(short v);

template <int FRAC>
__device__ inline short float2fixed(float v);

template <int FRAC>
__device__ inline float fixed2float8(int8_t v);

template <int FRAC>
__device__ inline int8_t float2fixed8(float v);

}

#include <cudatl/impl/fixed.hpp>

#endif
