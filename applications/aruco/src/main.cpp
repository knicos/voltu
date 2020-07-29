#include <ftl/timer.hpp>
#include <ftl/configuration.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <vector>
#include <string>

int main(int argc, char** argv) {
	std::vector<cv::Mat> tags;
	unsigned int ntags = 10;
	cv::Ptr<cv::aruco::Dictionary> dict =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	unsigned int size = 1024;
	unsigned int margin = 1;
	unsigned int delay = 100;

	argc--;
	argv++;
	auto opts = ftl::config::read_options(&argv, &argc);

	if (opts.count("delay"))
		delay = std::stoi(opts["delay"]);
	if (opts.count("dict"))
		dict = cv::aruco::getPredefinedDictionary(std::stoi(opts["dict"]));
	if (opts.count("ntags"))
		ntags = std::stoi(opts["ntags"]);
	if (opts.count("size"))
		size = std::stoi(opts["size"]);

	for (unsigned int i = 0; i < ntags; i++) {
		cv::aruco::drawMarker(dict, i, size, tags.emplace_back(), margin);
	}

	int id = 0;
	ftl::timer::setInterval(delay);
	ftl::timer::setHighPrecision(true);
	auto h = ftl::timer::add(ftl::timer::kTimerMain, [&](uint64_t){

		cv::imshow("ArUco", tags[id]);
		if (cv::waitKey(1) == 27) { ftl::timer::stop(false); }
		id = (id + 1) % ntags;
		return true;
	});

	ftl::timer::start(true);

	return 0;
}