#include "parsers.hpp"

#include <loguru.hpp>

#include <ftl/codecs/channels.hpp>
#include <tuple>

ftl::rgbd::Camera ftl::stream::parseCalibration(const ftl::codecs::Packet &pkt) {
	std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, unsigned int> params;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(params);

	LOG(INFO) << "Got Calibration: "
			  << std::get<0>(params).width << "x" << std::get<0>(params).height
			  << ", fx: " << std::get<0>(params).fx
			  << ", cx: " << std::get<0>(params).cx
			  << ", cy: " << std::get<0>(params).cy;
	
	return std::get<0>(params);
}

Eigen::Matrix4d ftl::stream::parsePose(const ftl::codecs::Packet &pkt) {
	Eigen::Matrix4d p;

	if (pkt.codec == ftl::codecs::codec_t::POSE) {
		p = Eigen::Map<Eigen::Matrix4d>((double*)pkt.data.data());
	} else if (pkt.codec == ftl::codecs::codec_t::MSGPACK) {

		auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
		std::vector<double> posevec;
		unpacked.get().convert(posevec);
		p = Eigen::Matrix4d(posevec.data());
		
	}
	return p;
}

std::string ftl::stream::parseConfig(const ftl::codecs::Packet &pkt) {
	std::string cfg;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(cfg);

	LOG(INFO) << "Config Received: " << cfg;
	return cfg;
}
