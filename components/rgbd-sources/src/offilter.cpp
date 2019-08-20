#include <loguru.hpp>

#include "ftl/offilter.hpp"

using namespace ftl::rgbd;

using cv::Mat;
using cv::Size;

using std::vector;

template<typename T> static bool inline isValidDisparity(T d) { return (0.0 < d) && (d < 256.0); } // TODO

OFDisparityFilter::OFDisparityFilter(Size size, int n_frames, float threshold) :
	n_(0), n_max_(n_frames), threshold_(threshold), size_(size)
{
	disp_ = Mat::zeros(size, CV_64FC(n_frames));
	gray_ = Mat::zeros(size, CV_8UC1);

	nvof_ = cv::cuda::NvidiaOpticalFlow_1_0::create(size.width, size.height,
													cv::cuda::NvidiaOpticalFlow_1_0::NV_OF_PERF_LEVEL_SLOW,
													true, false, false, 0);
	
}

void OFDisparityFilter::filter(Mat &disp, const Mat &gray)
{

	const int n = n_;
	n_ = (n_ + 1) % n_max_;
	
	nvof_->calc(gray, gray_, flowxy_);
	nvof_->upSampler(	flowxy_, size_.width, size_.height,
						nvof_->getGridSize(), flowxy_up_);

	CHECK(disp.type() == CV_32FC1);
	CHECK(gray.type() == CV_8UC1);
	CHECK(flowxy_up_.type() == CV_32FC2);

	gray.copyTo(gray_);

	vector<float> values(n_max_);

	for (int y = 0; y < size_.height; y++)
	{
		float *D = disp_.ptr<float>(y);
		float *d = disp.ptr<float>(y);
		float *flow = flowxy_up_.ptr<float>(y);

		for (int x = 0; x < size_.width; x++)
		{
			const float flow_l1 = abs(flow[2*x]) + abs(flow[2*x + 1]);

			if (flow_l1 < threshold_)
			{
				values.clear();

				if (isValidDisparity(d[x]))
				{
					bool updated = false;
					for (int i = 0; i < n_max_; i++)
					{
						float &val = D[n_max_ * x + (n_max_ - i + n) % n_max_];
						if (!isValidDisparity(val))
						{
							val = d[x];
							updated = true;
						}
					}
					if (!updated) { D[n_max_ * x + n] = d[x]; }
				}

				for (int i = 0; i < n_max_; i++)
				{
					float &val = D[n_max_ * x + i];
					if (isValidDisparity(val)) { values.push_back(val); }
				}

				if (values.size() > 0) {
					const auto median_it = values.begin() + values.size() / 2;
					std::nth_element(values.begin(), median_it , values.end());
					d[x] = *median_it;
				}

				/*
				if (isValidDepth(d[x]) && isValidDepth(D[x]))
				{
					D[x] = D[x] * 0.66 + d[x] * (1.0 - 0.66);
				}
				if (isValidDepth(D[x]))
				{
					d[x] = D[x];
				}
				else
				{
					D[x] = d[x];
				}
				*/
			}
			else
			{
				for (int i = 0; i < n_max_; i++)
				{
					D[n_max_ * x + i] = 0.0;
				}
			}
		}
	}
}
