#include <ftl/codecs/channels.hpp>

#include <opencv2/opencv.hpp>

struct ChannelInfo {
	const char *name;
	int type;
};

static ChannelInfo info[] = {
    "Colour", CV_8UC4,			// 0
    "Depth", CV_32F,			// 1
    "Right", CV_8UC4,			// 2
    "DepthRight", CV_32F,		// 3
    "Deviation", CV_32F,		// 4
    "Normals", CV_32FC4,		// 5
    "Weights", CV_16SC1,		// 6
    "Confidence", CV_32F,		// 7
    "EnergyVector", CV_32FC4,	// 8
    "Flow", CV_32F,				// 9
    "Energy", CV_32F,			// 10
	"Mask", CV_32S,				// 11
	"Density", CV_32F,			// 12
    "Support1", CV_8UC4,		// 13
    "Support2", CV_8UC4,		// 14
    "Segmentation", CV_32S,		// 15

	"ColourNormals", 0,			// 16
	"ColourHighRes", 0,			// 17
	"Disparity", 0,				// 18
	"Smoothing", 0,				// 19
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
