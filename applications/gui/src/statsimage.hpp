#ifndef _FTL_GUI_STATISTICSIMAGE_HPP_
#define _FTL_GUI_STATISTICSIMAGE_HPP_

#include <opencv2/core/mat.hpp>

namespace ftl {
namespace gui {

class StatisticsImage {
private:
	cv::Mat data_;	// CV_32FC3, channels: m, s, f
	cv::Size size_;	// image size
	float n_;		// total number of samples

public:
	explicit StatisticsImage(cv::Size size);
	StatisticsImage(cv::Size size, float max_f);

	/* @brief reset all statistics to 0
	 */
	void reset();

	/* @brief update statistics with new values
	 */
	void update(const cv::Mat &in);
	
	/* @brief variance (depth)
	 */
	void getVariance(cv::Mat &out);

	/* @brief standard deviation (depth)
	 */
	void getStdDev(cv::Mat &out);
	
	/* @brief mean value (depth)
	 */
	void getMean(cv::Mat &out);

	/* @brief percent of samples having valid depth value
	 */
	void getValidRatio(cv::Mat &out);
};

}
}

#endif  // _FTL_GUI_STATISTICSIMAGE_HPP_