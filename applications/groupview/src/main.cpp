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

static void from_json(nlohmann::json &json, map<string, Matrix4d> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4d m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

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

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "viewer_default");
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
		s->setChannel(ftl::rgbd::kChanDepth);
		group.addSource(s);
	}

	bool grab = false;

	group.sync([&grab](const ftl::rgbd::FrameSet &fs) {
		LOG(INFO) << "Complete set: " << fs.timestamp;
		if (grab) {
			grab = false;

#ifdef HAVE_LIBARCHIVE
			char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			auto writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");

			for (size_t i=0; i<fs.sources.size(); ++i) {
				writer.addCameraParams(std::string("camera")+std::to_string(i), fs.sources[i]->getPose(), fs.sources[i]->parameters());
				LOG(INFO) << "SAVE: " << fs.channel1[i].cols << ", " << fs.channel2[i].type();
				writer.addCameraRGBD(std::string("camera")+std::to_string(i), fs.channel1[i], fs.channel2[i]);
			}
#endif  // HAVE_LIBARCHIVE
		}
		return true;
	});

	int current = 0;

	while (ftl::running) {
		//std::this_thread::sleep_for(std::chrono::milliseconds(20));
		for (auto s : sources) s->grab(30);
		cv::Mat rgb,depth;
		sources[current%sources.size()]->getFrames(rgb, depth);
		if (!rgb.empty()) cv::imshow("View", rgb);
		auto key = cv::waitKey(20);

		if (key == 27) break;
		if (key == 'n') current++;
		if (key == 'g') grab = true;
	}

	return 0;
}
