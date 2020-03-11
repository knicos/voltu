namespace ftl { namespace cuda { namespace device
{
    namespace disp_bilateral_filter
    {
        template<typename T, typename C>
        void disp_bilateral_filter(cv::cuda::PtrStepSz<T> disp, cv::cuda::PtrStepSz<T> dispout, cv::cuda::PtrStepSz<C> img, int iters, const float *, size_t, int radius, T edge_disc, T max_disc, cudaStream_t stream);
    }
}}}
