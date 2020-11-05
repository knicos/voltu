template <int FRAC>
__device__ inline float cudatl::fixed2float(short v)
{
    return float(v) / float(1 << FRAC);
}

template <int FRAC>
__device__ inline short cudatl::float2fixed(float v)
{
    return short(v * float(1 << FRAC));
}

template <int FRAC>
__device__ inline float cudatl::fixed2float8(int8_t v)
{
    return float(v) / float(1 << FRAC);
}

template <int FRAC>
__device__ inline int8_t cudatl::float2fixed8(float v)
{
    return int8_t(v * float(1 << FRAC));
}
