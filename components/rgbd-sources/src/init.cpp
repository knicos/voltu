#include <ftl/data/new_frame.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/channels.hpp>

using ftl::codecs::Channel;
using ftl::data::StorageMode;

bool ftl_video_init =
	ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT) &&
	ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Pose, "pose", ftl::data::StorageMode::PERSISTENT) &&
	ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration2, "calibration_right", ftl::data::StorageMode::PERSISTENT); // &&
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Name, "name", ftl::data::StorageMode::PERSISTENT);

bool ftl_video_initialised() {
	return ftl_video_init;
}
