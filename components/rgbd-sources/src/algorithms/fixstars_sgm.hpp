/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_
#define _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <libsgm.h>
#include "../disparity.hpp"
#include <opencv2/cudastereo.hpp>
#include <ftl/configuration.hpp>

#include "ftl/cb_segmentation.hpp"

namespace ftl {
	namespace algorithms {

		/**
		 * Fixstars libSGM stereo matcher.
		 * @see https://github.com/fixstars/libSGM
		 *
		 * NOTE: We are using a modified version that supports disparity of 256.
		 * @see https://github.com/knicos/libSGM
		 */
		class FixstarsSGM : public ftl::rgbd::detail::Disparity {
		public:
			explicit FixstarsSGM(nlohmann::json &config);

			void compute(const cv::cuda::GpuMat &l, const cv::cuda::GpuMat &r, cv::cuda::GpuMat &disp, cv::cuda::Stream &stream) override;
			void setMask(cv::Mat &mask) override;

			/* Factory creator */
			static inline Disparity *create(ftl::Configurable *p, const std::string &name) {
				return ftl::create<FixstarsSGM>(p, name);
			}

		private:
			void init(const cv::Size size);

			float uniqueness_;
			int P1_;
			int P2_;
			cv::Size size_;
			bool use_filter_;
			cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
			sgm::StereoSGM *ssgm_;
			cv::cuda::GpuMat lbw_;
			cv::cuda::GpuMat rbw_;
			cv::cuda::GpuMat dispt_;

			cv::cuda::GpuMat l_downscaled_;
			cv::cuda::GpuMat dispt_full_res_;
		};
	};
};

#endif  // _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

