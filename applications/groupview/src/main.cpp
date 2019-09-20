#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>

#ifdef HAVE_LIBARCHIVE
#include <ftl/rgbd/snapshot.hpp>
#endif

#include <fstream>

using Eigen::Matrix4d;
using std::map;
using std::string;
using std::vector;
using cv::Size;
using cv::Mat;
using ftl::rgbd::Channel;

// TODO: remove code duplication (function from reconstruction)
static void from_json(nlohmann::json &json, map<string, Matrix4d> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4d m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

// TODO: remove code duplication (function from reconstruction)
static bool loadTransformations(const string &path, map<string, Matrix4d> &data) {
	std::ifstream file(path);
	if (!file.is_open()) {
		LOG(ERROR) << "Error loading transformations from file " << path;
		return false;
	}
	
	nlohmann::json json_registration;
	file >> json_registration;
	from_json(json_registration, data);
	return true;
}

// TODO: remove code duplication (function from calibrate-multi)
void stack(const vector<Mat> &img, Mat &out, const int rows, const int cols) {
	Size size = img[0].size();
	Size size_out = Size(size.width * cols, size.height * rows);
	if (size_out != out.size() || out.type() != CV_8UC3) {
		out = Mat(size_out, CV_8UC3, cv::Scalar(0, 0, 0));
	}

	for (size_t i = 0; i < img.size(); i++) {
		int row = i % rows;
		int col = i / rows;
		auto rect = cv::Rect(size.width * col, size.height * row, size.width, size.height);
		img[i].copyTo(out(rect));
	}
}

// TODO: remove code duplication (function from calibrate-multi)
void stack(const vector<Mat> &img, Mat &out) {
	// TODO
	int rows = 2;
	int cols = (img.size() + 1) / 2;
	stack(img, out, rows, cols);
}

void modeLeftRight(ftl::Configurable *root) {
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<ftl::rgbd::Source>(root, "sources", net);
	const string path = root->value<string>("save_to", "./");
	const string file_type = root->value<string>("file_type", "jpg");

	const size_t n_cameras = sources.size() * 2;
	ftl::rgbd::Group group;

	for (auto* src : sources) {
		src->setChannel(Channel::Right);
		group.addSource(src);
	}

	std::mutex mutex;
	std::atomic<bool> new_frames = false;
	vector<Mat> rgb(n_cameras), rgb_new(n_cameras);
	
	group.sync([&mutex, &new_frames, &rgb_new](ftl::rgbd::FrameSet &frames) {
		mutex.lock();
		bool good = true;
		for (size_t i = 0; i < frames.frames.size(); i ++) {
			auto &chan1 = frames.frames[i].get<cv::Mat>(Channel::Colour);
			auto &chan2 = frames.frames[i].get<cv::Mat>(frames.sources[i]->getChannel());
			if (chan1.empty()) good = false;
			if (chan2.empty()) good = false;
			if (chan1.channels() != 3) good = false; // ASSERT
			if (chan2.channels() != 3) good = false;
			if (!good) break;
			
			chan1.copyTo(rgb_new[2 * i]);
			chan2.copyTo(rgb_new[2 * i + 1]);
		}

		new_frames = good;
		mutex.unlock();
		return true;
	});
	
	int idx = 0;

	Mat show;
	
	while (ftl::running) {
		int key;
		
		while (!new_frames) {
			for (auto src : sources) { src->grab(30); }
			key = cv::waitKey(10);
		}

		mutex.lock();
		rgb.swap(rgb_new);
		new_frames = false;
		mutex.unlock();
		
		stack(rgb, show);
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);

		key = cv::waitKey(100);
		// TODO: fix
		if (key == 's') {
			 for (size_t c = 0; c < n_cameras; c++ ) {
			 	cv::imwrite(path + "camera" + std::to_string(c) + "_" + std::to_string(idx) + "." + file_type, rgb[c]);
			 }
			 LOG(INFO) << "Saved (" << idx << ")";
			 idx++;
		}
		if (key == 27) break;
	}
}

void modeFrame(ftl::Configurable *root, int frames=1) {
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<ftl::rgbd::Source>(root, "sources", net);

	std::map<std::string, Eigen::Matrix4d> transformations;
	if (loadTransformations(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json", transformations)) {
		LOG(INFO) << "Loaded camera trasformations from file";
	}
	else {
		LOG(ERROR) << "Error loading camera transformations from file";
	}

	ftl::rgbd::Group group;
	for (auto s : sources) {
		string uri = s->getURI();
		auto T = transformations.find(uri);
		if (T == transformations.end()) {
			LOG(ERROR) << "Camera pose for " + uri + " not found in transformations";
		} else {
			s->setPose(T->second);
		}
		s->setChannel(Channel::Depth);
		group.addSource(s);
	}

	std::atomic<bool> grab = false;
	std::atomic<bool> video = false;

	vector<cv::Mat> rgb(sources.size());
	vector<cv::Mat> depth(sources.size());
	MUTEX mtx;

	group.sync([&grab,&rgb,&depth,&mtx](ftl::rgbd::FrameSet &fs) {
		UNIQUE_LOCK(mtx, lk);
		//LOG(INFO) << "Complete set: " << fs.timestamp;
		if (!ftl::running) { return false; }
		
		std::vector<cv::Mat> frames;

		for (size_t i=0; i<fs.sources.size(); ++i) {
			auto &chan1 = fs.frames[i].get<cv::Mat>(Channel::Colour);
			auto &chan2 = fs.frames[i].get<cv::Mat>(fs.sources[i]->getChannel());
			if (chan1.empty() || chan2.empty()) return true;

			frames.push_back(chan1);
		}

		cv::Mat show;

		stack(frames, show);

		cv::resize(show, show, cv::Size(1280,720));
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);

		auto key = cv::waitKey(1);
		if (key == 27) ftl::running = false;
		if (key == 'g') grab = true;

#ifdef HAVE_LIBARCHIVE
		if (grab) {
			grab = false;
			char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			auto writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");

			for (size_t i=0; i<fs.sources.size(); ++i) {
				auto &chan1 = fs.frames[i].get<cv::Mat>(Channel::Colour);
				auto &chan2 = fs.frames[i].get<cv::Mat>(fs.sources[i]->getChannel());

				writer.addSource(fs.sources[i]->getURI(), fs.sources[i]->parameters(), fs.sources[i]->getPose());
				//LOG(INFO) << "SAVE: " << fs.channel1[i].cols << ", " << fs.channel2[i].type();
				writer.addRGBD(i, chan1, chan2);
			}
		}
#endif  // HAVE_LIBARCHIVE
		return true;
	});

	/*cv::Mat show;

	while (ftl::running) {
		for (auto s : sources) s->grab(30);
		for (size_t i = 0; i < sources.size(); i++) {
			//do { sources[i]->getFrames(rgb[i], depth[i]); }
			while(rgb[i].empty());
		}

		{
			UNIQUE_LOCK(mtx, lk);
			stack(rgb, show);
		}
		cv::resize(show, show, cv::Size(1280,720));
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);

		auto key = cv::waitKey(20);
		if (key == 27) break;
		if (key == 'g') grab = true;
	}*/
}

void modeVideo(ftl::Configurable *root) {

	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<ftl::rgbd::Source>(root, "sources", net);
	const string path = root->value<string>("save_to", "./");

	for (auto* src : sources) { src->setChannel(Channel::Depth); }

	cv::Mat show;
	vector<cv::Mat> rgb(sources.size());
	vector<cv::Mat> depth(sources.size());

#ifdef HAVE_LIBARCHIVE
	char timestamp[18];
	std::time_t t=std::time(NULL);
	std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
	ftl::rgbd::SnapshotWriter writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");

	for (size_t i = 0; i < sources.size(); i++) {
		writer.addSource(sources[i]->getURI(), sources[i]->parameters(), sources[i]->getPose());
	}
#endif // HAVE_LIBARCHIVE

	bool save = false;

	while (ftl::running) {
		for (auto s : sources) s->grab(30);
		for (size_t i = 0; i < sources.size(); i++) {
			do { sources[i]->getFrames(rgb[i], depth[i]); }
			while(rgb[i].empty() || depth[i].empty());
		}

#ifdef HAVE_LIBARCHIVE
		if (save) {
			for (size_t i = 0; i < sources.size(); i++) {
				writer.addRGBD(i, rgb[i], depth[i]);
			}
		}
#endif // HAVE_LIBARCHIVE

		stack(rgb, show);
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);

		auto key = cv::waitKey(20);
		if (key == 'r') {
			save = true;
		}
		if (key == 's') {
			save = false;
		}
		if (key == 27) break;
	}
}

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "viewer_default");

	if (root->value("stereo", false)) {
		LOG(INFO) << "Stereo images mode";
		modeLeftRight(root);
	} else if (root->value("video", false)) {
		LOG(INFO) << "Video mode";
		modeVideo(root);
	} else {
		//modeVideo(root);
		modeFrame(root);
	}

	ftl::running = false;
	return 0;
}
