#ifndef _FTL_LOCAL_HPP_
#define _FTL_LOCAL_HPP_

#include <string>
#include <nlohmann/json.hpp>

namespace cv {
	class Mat;
	class VideoCapture;
};

namespace ftl {
class LocalSource {
	public:
	LocalSource(nlohmann::json &config);
	LocalSource(const std::string &vid, nlohmann::json &config);
	
	bool left(cv::Mat &m);
	bool right(cv::Mat &m);
	bool get(cv::Mat &l, cv::Mat &r);
	
	//void setFramerate(float fps);
	//float getFramerate() const;
	
	double getTimestamp() const;
	
	bool isStereo() const;
	
	private:
	double timestamp_;
	bool stereo_;
	//float fps_;
	bool flip_;
	bool nostereo_;
	float downsize_;
	cv::VideoCapture *camera_a_;
	cv::VideoCapture *camera_b_;
};
};

#endif // _FTL_LOCAL_HPP_

