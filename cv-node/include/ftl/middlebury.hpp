#ifndef _FTL_MIDDLEBURY_HPP_
#define _FTL_MIDDLEBURY_HPP_

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <string>

namespace ftl {
namespace middlebury {
	void test(nlohmann::json &config);
	
	void evaldisp(const cv::Mat &disp, const cv::Mat &gtdisp,
			const cv::Mat &mask, float badthresh, int maxdisp, int rounddisp);
			
	void readFilePFM(cv::Mat &img, const std::string &filename);
	void writeFilePFM(const cv::Mat &img, const char* filename, float scalefactor=1/255.0);
}
}

#endif // _FTL_MIDDLEBURY_HPP_

