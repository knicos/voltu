#ifndef _FTL_OPERATORS_TRACKING_HPP_
#define _FTL_OPERATORS_TRACKING_HPP_

#include "ftl/operators/operator.hpp"
#include <opencv2/objdetect.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/aruco.hpp>

#include <ftl/threads.hpp>

namespace ftl {
namespace operators {

/**
 * Object detection and tracking.
 *
 * cv::CascadeClassifier used in detection
 * https://docs.opencv.org/master/d1/de5/classcv_1_1CascadeClassifier.html
 *
 * cv::TrackerKCF or cv::TrackerCSRT is used for tracking
 * https://docs.opencv.org/master/d2/dff/classcv_1_1TrackerKCF.html
 * https://docs.opencv.org/master/d2/da2/classcv_1_1TrackerCSRT.html
 *
 * Configurable parameters:
 * - max_distance: If classifier detects an object closer than max_distance, the
 *   found object is considered already tracked and new tracker is not created.
 *   Value in pixels.
 * - update_bounding_box: When true, tracker is always re-initialized with new
 *   bounding box when classifier detects already tracked object.
 * - max_tarcked: Maximum number of tracked objects.
 * - max_fail: Number of tracking failures before object is removed. Value in
 *   frames.
 * - detect_n_frames: How often object detection is performed.
 * - filename: Path to OpenCV CascadeClassifier file
 * - scale_neighbors, scalef, min_size, max_size: OpenCV CascadeClassifier
 *   parameters
 * - tracker_type: KCF or CSRT
 * - debug: Draw bounding boxes and object IDs in image.
 */
class DetectAndTrack : public ftl::operators::Operator {
	public:
	explicit DetectAndTrack(ftl::operators::Graph *g, ftl::Configurable*);
	~DetectAndTrack() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	void wait(cudaStream_t) override;

	static void configuration(ftl::Configurable*) {}

	protected:
	bool init();

	bool detect(const cv::Mat &im);
	bool track(const cv::Mat &im);

	private:
	std::future<bool> job_;
	std::future<bool> detect_job_;
	bool detecting_;
	MUTEX mtx_;

	int createTracker(const cv::Mat &im, const cv::Rect2d &obj);

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
	std::list<Object> tracked_;

	// 0: KCF, 1: CSRT
	int tracker_type_;

	// update tracked bounding box when detected
	bool update_bounding_box_;
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

/*
 * AruCo tag detection and tracking.
 *
 * Configurable parameters:
 * - dictionary: ArUco dictionary id. See PREDEFINED_DICTIONARY_NAME in
 *   opencv2/aruco/dictionary.hpp for possible values.
 * - marker_size: Marker size (side), in meteres.
 * - estimate_pose: Estimate marker poses. Requires marker_size to be set.
 * - debug: When enabled, markers and poses are drawn to image.
 */
class ArUco : public ftl::operators::Operator {
	public:
	explicit ArUco(ftl::operators::Graph *g, ftl::Configurable*);
	~ArUco() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;
	virtual void wait(cudaStream_t) override;

	ftl::codecs::Channel channel_in_;
	ftl::codecs::Channel channel_out_;

	static void configuration(ftl::Configurable*) {}

	private:
	bool estimate_pose_;
	float marker_size_;
	cv::Mat tmp_;
	std::future<void> job_;

	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Ptr<cv::aruco::DetectorParameters> params_;
};

}
}

#endif // _FTL_OPERATORS_TRACKING_HPP_
