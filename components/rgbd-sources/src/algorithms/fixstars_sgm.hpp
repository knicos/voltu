/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_
#define _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>

#include "../disparity.hpp"
#include <ftl/configuration.hpp>
#include <ftl/config.h>

#include <libsgm.h>
#include "ftl/offilter.hpp"

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

			void compute(ftl::rgbd::Frame &frame, cv::cuda::Stream &stream) override;
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
			bool use_filter_;
			bool use_off_;
			cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
			sgm::StereoSGM *ssgm_;
			cv::cuda::GpuMat lbw_;
			cv::cuda::GpuMat rbw_;
			cv::cuda::GpuMat dispt_;

			#ifdef HAVE_OPTFLOW
			ftl::rgbd::OFDisparityFilter off_;
			#endif
		};
	};
};

#endif  // _FTL_ALGORITHMS_FIXSTARS_SGM_HPP_

