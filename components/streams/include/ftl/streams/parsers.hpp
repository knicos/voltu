#ifndef _FTL_STREAM_PARSERS_HPP_
#define _FTL_STREAM_PARSERS_HPP_

#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/packet.hpp>
#include <Eigen/Eigen>

namespace ftl {
namespace stream {

Eigen::Matrix4d parsePose(const ftl::codecs::Packet &pkt);
ftl::rgbd::Camera parseCalibration(const ftl::codecs::Packet &pkt);
std::string parseConfig(const ftl::codecs::Packet &pkt);

}
}

#endif  // _FTL_STREAM_PARSERS_HPP_
