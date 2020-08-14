#include "collisions.hpp"
#include <ftl/codecs/touch.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/algorithms/dbscan.hpp>

using ftl::codecs::Channel;
using ftl::rgbd::Capability;

void ftl::render::collision2touch(const ftl::rgbd::Frame &rgbdframe,
 const std::vector<float4> &collisions,
 const std::list<ftl::data::FrameSetPtr> &sets, uint32_t myid, float tmin, float tmax) {

	std::vector<float4> clusters;
	std::vector<short> labels;
	ftl::dbscan<float4>(collisions, [](const std::vector<float4> &pts, size_t idx, float radius) {
		std::vector<size_t> neighbors;
		for (auto i = 0u; i < pts.size(); i++) {
			if (i == idx) {
				continue;
			}
			float dx = pts[idx].x - pts[i].x;
			float dy = pts[idx].y - pts[i].y;

			if (dx*dx+dy*dy < radius*radius) {
				neighbors.push_back(i);
			}
		}
		return neighbors;
	}, 5, 16.0f, labels, clusters);

	// TODO: Support multi-touch
	if (clusters.size() == 1) {
		//LOG(INFO) << "Found " << clusters.size() << " collisions";
		//LOG(INFO) << "  -- " << clusters[0].x << "," << clusters[0].y << " " << clusters[0].z;

		// Find all frames that support touch
		for (auto &s : sets) {
			if (s->frameset() == myid) continue;

			for (const auto &f : s->frames) {
				if (f.has(Channel::Capabilities)) {
					const auto &cap = f.get<std::unordered_set<Capability>>(Channel::Capabilities);

					// If it supports touch, calculate the touch points in screen coordinates
					if (cap.count(Capability::TOUCH)){
						const auto &rgbdf = f.cast<ftl::rgbd::Frame>();

						// TODO: Use Eigen directly.
						auto fpose = rgbdf.getPose();
						if (s->hasChannel(Channel::Pose)) {
							fpose = s->cast<ftl::rgbd::Frame>().getPose() * fpose;
						}
						auto pose = MatrixConversion::toCUDA((fpose.inverse() * rgbdframe.getPose()).cast<float>());
						float3 campos = pose * rgbdframe.getLeft().screenToCam(clusters[0].x, clusters[0].y, clusters[0].z);
						const auto &cam = rgbdf.getLeft();
						int2 pt = cam.camToScreen<int2>(campos);
						//LOG(INFO) << "TOUCH AT " << pt.x << "," << pt.y << " - " << campos.z;

						{
							// Send the touch data
							auto response = f.response();
							auto &touches = response.create<std::vector<ftl::codecs::Touch>>(Channel::Touch);
							auto &touch = touches.emplace_back();
							touch.id = 0;
							touch.x = pt.x;
							touch.y = pt.y;
							touch.type = ftl::codecs::TouchType::COLLISION;
							touch.strength = (std::abs(campos.z - cam.maxDepth) <= tmin) ? 255 : 0;
							touch.d = campos.z;
						}
					}
				}
			}
		}
	}
}