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
	unsigned int size = 512;
	unsigned int margin = 64;
	unsigned int delay = 50;

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
	if (opts.count("margin"))
		margin = std::stoi(opts["margin"]);

	cv::Mat blank = cv::Mat(size + margin*2, size + margin*2, CV_8UC1);
	blank.setTo(255);

	for (unsigned int i = 0; i < ntags; i++) {
		auto& tag = tags.emplace_back();
		tag.create(size + margin*2, size + margin*2, CV_8UC1);
		tag.setTo(255);
		cv::aruco::drawMarker(dict, i, size, tag(cv::Rect(margin, margin, size, size)), 1);
	}

	int id = 0;
	bool show_blank = false;
	ftl::timer::setInterval(delay);
	ftl::timer::setHighPrecision(true);
	auto h = ftl::timer::add(ftl::timer::kTimerMain, [&](uint64_t){
		cv::imshow("ArUco", show_blank ? blank : tags[id]);
		if (cv::waitKey(1) == 27) { ftl::timer::stop(false); }
		show_blank = !show_blank;
		id = (id + 1) % ntags;
		return true;
	});

	ftl::timer::start(true);

	return 0;
}