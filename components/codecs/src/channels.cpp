#include <ftl/codecs/channels.hpp>
#include <unordered_map>
#include <opencv2/opencv.hpp>

using ftl::codecs::Channel;

struct ChannelInfo {
	const char *name;
	int type;
};

static const std::unordered_map<Channel,ChannelInfo> info = {
    {Channel::Colour, {"Left", CV_8UC4}},
	{Channel::Depth, {"Depth", CV_32F}},
	{Channel::Right, {"Right", CV_8UC4}},
	{Channel::Depth2, {"Depth Right", CV_32F}},
	{Channel::Deviation, {"Deviation", CV_32F}},
	{Channel::Normals, {"Normals", CV_32FC4}},
	{Channel::Weights, {"Weights", CV_32F}},
	{Channel::Confidence, {"Confidence", CV_32F}},
	{Channel::EnergyVector, {"Energy Vector", CV_32FC4}},
	{Channel::Flow, {"Flow", CV_32F}},
	{Channel::Energy, {"Energy", CV_32F}},
	{Channel::Mask, {"Mask", CV_8U}},
	{Channel::Density, {"Density", CV_32F}},
	{Channel::Support1, {"Support1", CV_8UC4}},
	{Channel::Support2, {"Support2", CV_8UC4}},
	{Channel::Segmentation, {"Segmentation", CV_8U}},
	{Channel::Normals2, {"Normals Right", CV_32FC4}},
	{Channel::UNUSED1, {"Unused", CV_8UC4}},
	{Channel::Disparity, {"Disparity", CV_16S}},
	{Channel::Smoothing, {"Smoothing", CV_32F}},
	{Channel::UNUSED2, {"Unused", CV_8UC4}},
	{Channel::Overlay, {"Overlay", CV_8UC4}},
	{Channel::GroundTruth, {"Ground Truth", CV_32F}},

	{Channel::AudioMono, {"Audio (Mono)", -1}},
	{Channel::AudioStereo, {"Audio (Stereo)", -1}},

	{Channel::Configuration, {"Configuration", -1}},
	{Channel::Calibration, {"Calibration", -1}},
	{Channel::Pose, {"Pose", -1}},
	{Channel::Calibration2, {"Calibration High-res", -1}},
	{Channel::MetaData, {"Meta Data", -1}},
	{Channel::Capabilities, {"Capabilities", -1}},
	{Channel::CalibrationData, {"Calibration Data", -1}},
	{Channel::Thumbnail, {"Thumbnail", -1}},

	{Channel::Data, {"Generic Data", -1}},
	{Channel::Faces, {"Faces", -1}},
	{Channel::Shapes3D, {"Shapes 3D", -1}},
	{Channel::Messages, {"Messages", -1}},
	{Channel::Touch, {"Touch", -1}}
};

std::string ftl::codecs::name(Channel c) {
	if (c == Channel::None) return "None";
	auto i = info.find(c);
	if (i != info.end()) {
		return i->second.name;
	} else {
		return "Unknown";
	}
}

int ftl::codecs::type(Channel c)  {
	if (c == Channel::None) return 0;
	auto i = info.find(c);
	if (i != info.end()) {
		return i->second.type;
	} else {
		return -1;
	}
}
