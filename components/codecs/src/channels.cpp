#include <ftl/codecs/channels.hpp>

#include <opencv2/opencv.hpp>

struct ChannelInfo {
	const char *name;
	int type;
};

static ChannelInfo info[] = {
    "Colour", CV_8UC4,
    "Depth", CV_32F,
    "Right", CV_8UC4,
    "DepthRight", CV_32F,
    "Deviation", CV_32F,
    "Normals", CV_32FC4,
    "Points", CV_32FC4,
    "Confidence", CV_32F,
    "EnergyVector", CV_32FC4,
    "Flow", CV_32F,
    "Energy", CV_32F,
	"Mask", CV_32S,
	"Density", CV_32F,
    "Support1", CV_8UC4,
    "Support2", CV_8UC4,
    "Segmentation", CV_32S,

	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,

	"AudioLeft", 0,
	"AudioRight", 0,

	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,
	"NoName", 0,

	"Configuration", 0,
	"Calibration", 0,
	"Pose", 0,
	"Data", 0
};

std::string ftl::codecs::name(Channel c) {
	if (c == Channel::None) return "None";
	else return info[(int)c].name;
}

int ftl::codecs::type(Channel c)  {
	if (c == Channel::None) return 0;
	else return info[(int)c].type;
}
