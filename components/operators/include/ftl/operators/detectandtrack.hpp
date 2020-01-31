#ifndef _FTL_OPERATORS_CASCADECLASSIFIER_HPP_
#define _FTL_OPERATORS_CASCADECLASSIFIER_HPP_

#include "ftl/operators/operator.hpp"
#include <opencv2/objdetect.hpp>
#include <opencv2/tracking.hpp>

namespace ftl {
namespace operators {

/**
 * Object detection and tracking. 
 * 
 * cv::CascadeClassifier used in detection
 * https://docs.opencv.org/master/d1/de5/classcv_1_1CascadeClassifier.html
 * 
 * cv::TrackerKCF used in tracking
 * https://docs.opencv.org/master/d2/dff/classcv_1_1TrackerKCF.html
 * 
 */
class DetectAndTrack : public ftl::operators::Operator {
	public:
	explicit DetectAndTrack(ftl::Configurable*);
	~DetectAndTrack() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	protected:
	bool init();
	
	bool detect(const cv::Mat &im);
	bool track(const cv::Mat &im);

	private:

	ftl::codecs::Channel channel_in_;
	ftl::codecs::Channel channel_out_;

	bool debug_;
	int id_max_;
	
	struct Object {
		int id;
		cv::Rect2d object;
		cv::Ptr<cv::Tracker> tracker;
		int fail_count;
	};
	std::vector<Object> tracked_;

	// detection: if detected object is farther than max_distance_, new tracked
	// object is added
	double max_distance_;
	// maximum number of tracked objects (performance)
	int max_tracked_;
	// maximum number of successive failed trackings before object is removed
	int	max_fail_;
	// how often detection is performed
	int detect_n_frames_;

	// cascade classifier parameters, see OpenCV documentation
	std::string fname_;
	double scalef_;
	int min_neighbors_;
	// min_size_ and max_size_ relative
	std::vector<double> min_size_;
	std::vector<double> max_size_;

	int n_frame_;
	cv::Mat gray_;
	cv::CascadeClassifier classifier_;
};

}
}

#endif // _FTL_OPERATORS_CASCADECLASSIFIER_HPP_
