#include <ftl/net_source.hpp>
#include <vector>
#include <thread>
#include <chrono>

using ftl::rgbd::NetSource;
using ftl::net::Universe;
using std::string;
using ftl::rgbd::CameraParameters;
using std::mutex;
using std::unique_lock;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

static bool getCalibration(Universe &net, string src, ftl::rgbd::CameraParameters &p) {
	while(true) {
		auto buf = net.findOne<vector<unsigned char>>((string) src +"/calibration");
		if (buf) {
			memcpy((char*)&p, (*buf).data(), (*buf).size());
			
			if (sizeof(p) != (*buf).size()) {
				LOG(ERROR) << "Corrupted calibration";
				return false;
			}

			LOG(INFO) << "Calibration received: " << p.cx << ", " << p.cy << ", " << p.fx << ", " << p.fy;
			
			return true;
		} else {
			LOG(INFO) << "Could not get calibration, retrying";
			sleep_for(milliseconds(500));
		}
	}
}

NetSource::NetSource(nlohmann::json &config) : RGBDSource(config) {

}

NetSource::NetSource(nlohmann::json &config, ftl::net::Universe *net)
		: RGBDSource(config, net) {

	has_calibration_ = getCalibration(*net, config["uri"].get<string>(), params_);
	
	net->subscribe(config["uri"].get<string>(), [this](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
		unique_lock<mutex> lk(mutex_);
		cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb_);
		//Mat(rgb_.size(), CV_16UC1);
		cv::imdecode(d, cv::IMREAD_UNCHANGED, &depth_);
		depth_.convertTo(depth_, CV_32FC1, 1.0f/(16.0f*100.0f));
	});
}

NetSource::~NetSource() {
	// TODO Unsubscribe
}

void NetSource::grab() {
	// net_.broadcast("grab");
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
