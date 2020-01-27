namespace ftl { namespace cuda { namespace device
{
    namespace disp_bilateral_filter
    {
        template<typename T>
        void disp_bilateral_filter(cv::cuda::PtrStepSz<T> disp, cv::cuda::PtrStepSzb img, int channels, int iters, const float *, const float *, size_t, int radius, T edge_disc, T max_disc, cudaStream_t stream);
    }
}}}
