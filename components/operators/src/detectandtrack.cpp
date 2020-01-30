#include <cmath>

#include "loguru.hpp"
#include "ftl/operators/detectandtrack.hpp"

using std::string;
using std::vector;
using std::map;

using cv::Mat;
using cv::Size;
using cv::Rect;
using cv::Rect2d;
using cv::Point2i;
using cv::Point2d;

using ftl::rgbd::Frame;
using ftl::operators::DetectAndTrack;

DetectAndTrack::DetectAndTrack(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {
	init();
}

bool DetectAndTrack::init() {
	fname_ = config()->value<string>("filename", "");
	
	detect_n_frames_ = config()->value<int>("n_frames", 10);
	detect_n_frames_ = detect_n_frames_ < 0.0 ? 0.0 : detect_n_frames_;

	max_distance_ = config()->value<double>("max_distance", 100.0);
	max_distance_ = max_distance_ < 0.0 ? 0.0 : max_distance_;

	max_fail_ = config()->value<int>("max_fail", 10);
	max_fail_ = max_fail_ < 0 ? 10 : max_fail_;

	max_tracked_ = config()->value<int>("max_tracked", 3);
	max_tracked_ = max_tracked_ < 0 ? 10 : max_tracked_;

	scalef_ = config()->value<double>("scalef", 1.1);
	min_neighbors_ = config()->value<int>("min_neighbors", 3);

	auto min_size = config()->get<vector<double>>("min_size");
	auto max_size = config()->get<vector<double>>("max_size");

	if (min_size && min_size->size() == 2) { min_size_ = *min_size; }
	else { min_size_ = {0.0, 0.0}; }
	if (max_size && max_size->size() == 2) { max_size_ = *max_size; }
	else { max_size_ = {1.0, 1.0}; }
	
	min_size_[0] = max(min(1.0, min_size_[0]), 0.0);
	min_size_[1] = max(min(1.0, min_size_[1]), 0.0);
	min_size_[0] = max(min(1.0, max_size_[0]), 0.0);
	min_size_[1] = max(min(1.0, max_size_[1]), 0.0);
	if (min_size_[0] > max_size_[0]) { min_size_[0] = max_size_[0]; }
	if (min_size_[1] > max_size_[1]) { min_size_[1] = max_size_[1]; }

	channel_in_ = ftl::codecs::Channel::Colour;

	bool retval = false;

	try {
		retval = classifier_.load(fname_);
	}
	catch (cv::Exception &ex)
	{
		retval = false;
		LOG(ERROR) << ex.what();
	}

	if (!retval) {
		LOG(ERROR) << "can't load: " << fname_;
		return false;
	}

	return true;
}

static double distance(Point2i p, Point2i q) {
	double a = (p.x-q.x);
	double b = (p.y-q.y);
	return sqrt(a*a+b*b);
}

static Point2d center(Rect2d obj) {
	return Point2d(obj.x+obj.width/2.0, obj.y+obj.height/2.0);
}

bool DetectAndTrack::detect(const Mat &im) {
	Size min_size(im.size().width*min_size_[0], im.size().height*min_size_[1]);
	Size max_size(im.size().width*max_size_[0], im.size().height*max_size_[1]);

	vector<Rect> objects;

	classifier_.detectMultiScale(im, objects,
								 scalef_, min_neighbors_, 0, min_size, max_size);
	
	LOG(INFO) << "Cascade classifier found " << objects.size() << " objects";

	for (const Rect2d &obj : objects) {
		Point2d c = center(obj);

		bool found = false;
		for (auto &tracker : tracked_) {
			if (distance(center(tracker.object), c) < max_distance_) {
				// update? (bounding box can be quite different)
				// tracker.object = obj;
				found = true;
				break;
			}
		}

		if (!found && (tracked_.size() < max_tracked_)) {
			cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
			tracker->init(im, obj);
			tracked_.push_back({ tracker, obj, 0 });
		}
	}

	return true;
}

bool DetectAndTrack::track(const Mat &im) {
	for (auto it = tracked_.begin(); it != tracked_.end();) {
		if (!it->tracker->update(im, it->object)) {
			it->fail_count++;
		}
		else {
			it->fail_count = 0;
		}

		if (it->fail_count > max_fail_) {
			tracked_.erase(it);
		}
		else { it++; }
	}

	return true;
}

bool DetectAndTrack::apply(Frame &in, Frame &out, cudaStream_t stream) {
	if (classifier_.empty()) {
		LOG(ERROR) << "classifier not loaded";
		return false;
	}

	if (!in.hasChannel(channel_in_)) {
		LOG(ERROR) << "input channel missing";
		return false;
	}

	in.download(channel_in_);
	Mat im = in.get<Mat>(channel_in_);

	track(im);

	if ((n_frame_++ % detect_n_frames_ == 0) && (tracked_.size() < max_tracked_)) {
		if (im.channels() == 1) {
			gray_ = im;
		}
		else if (im.channels() == 4) {
			cv::cvtColor(im, gray_, cv::COLOR_BGRA2GRAY);
		}
		else if (im.channels() == 3) {
			cv::cvtColor(im, gray_, cv::COLOR_BGR2GRAY);
		}
		else {
			LOG(ERROR) << "unsupported number of channels in input image";
			return false;
		}

		detect(gray_);
	}

	// TODO: save results somewhere
	std::vector<Rect2d> result;
	result.reserve(tracked_.size());
	for (auto const &tracked : tracked_) { result.push_back(tracked.object); }
	in.create(ftl::codecs::Channel::Data, result);

	// TODO: should be uploaded by operator which requires data on GPU
	in.upload(channel_in_);

	return true;
}
