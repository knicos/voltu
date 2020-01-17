#include "reconstruction.hpp"

#include "ftl/operators/smoothing.hpp"
#include "ftl/operators/colours.hpp"
#include "ftl/operators/normals.hpp"
#include "ftl/operators/filling.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/antialiasing.hpp"
#include "ftl/operators/mvmls.hpp"
#include "ftl/operators/clipping.hpp"
#include <ftl/operators/disparity.hpp>

using ftl::Reconstruction;
using ftl::codecs::Channel;

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
	Eigen::Affine3d rx =
		Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d ry =
		Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
	Eigen::Affine3d rz =
		Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
	return rz * rx * ry;
}

Reconstruction::Reconstruction(nlohmann::json &config, const std::string name) :
	ftl::Configurable(config), busy_(false), rbusy_(false), new_frame_(false), fs_render_(), fs_align_() {
	gen_ = nullptr;

	//renderer_ = ftl::create<ftl::render::Triangular>(this, "renderer", &fs_render_);

	pipeline_ = ftl::config::create<ftl::operators::Graph>(this, "pre_filters");
	pipeline_->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel
	pipeline_->append<ftl::operators::DisparityBilateralFilter>("bilateral_filter")->set("enabled", false);
	pipeline_->append<ftl::operators::DisparityToDepth>("calculate_depth")->set("enabled", false);
	pipeline_->append<ftl::operators::ClipScene>("clipping")->set("enabled", false);
	pipeline_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	//pipeline_->append<ftl::operators::HFSmoother>("hfnoise");  // Remove high-frequency noise
	pipeline_->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	//pipeline_->append<ftl::operators::SmoothChannel>("smoothing");  // Generate a smoothing channel
	//pipeline_->append<ftl::operators::ScanFieldFill>("filling");  // Generate a smoothing channel
	pipeline_->append<ftl::operators::CrossSupport>("cross");
	pipeline_->append<ftl::operators::DiscontinuityMask>("discontinuity");
	pipeline_->append<ftl::operators::CrossSupport>("cross2")->set("discon_support", true);
	pipeline_->append<ftl::operators::CullDiscontinuity>("remove_discontinuity")->set("enabled", false);
	//pipeline_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
	pipeline_->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);
	pipeline_->append<ftl::operators::MultiViewMLS>("mvmls");

	//pipeline_->set("enabled", false);
}

Reconstruction::~Reconstruction() {
	// TODO delete
}

size_t Reconstruction::size() {
	UNIQUE_LOCK(exchange_mtx_, lk);
	return fs_align_.frames.size();
}


ftl::rgbd::FrameState &Reconstruction::state(int ix) {
	UNIQUE_LOCK(exchange_mtx_, lk);
	if (ix >= 0 && ix < fs_align_.frames.size()) {
		return *fs_align_.frames[ix].origin();
	}
	throw ftl::exception("State index out-of-bounds");
}

void Reconstruction::onFrameSet(const ftl::rgbd::VideoCallback &cb) {
	cb_ = cb;
}

bool Reconstruction::post(ftl::rgbd::FrameSet &fs) {
	pipeline_->apply(fs, fs, 0);
		
	{
		UNIQUE_LOCK(exchange_mtx_, lk);
		if (new_frame_ == true) LOG(WARNING) << "Frame lost";
		fs.swapTo(fs_align_);
		new_frame_ = true;
	}

	if (cb_) {
		ftl::pool.push([this](int id) {
			if (new_frame_) {
				{
					UNIQUE_LOCK(exchange_mtx_, lk);
					new_frame_ = false;
					fs_align_.swapTo(fs_render_);
				}

				if (cb_) cb_(fs_render_);
			}
		});
	}

	return true;
}

void Reconstruction::setGenerator(ftl::rgbd::Generator *gen) {
	if (gen_) {
		throw ftl::exception("Reconstruction already has generator");
	}

	gen_ = gen;
	gen_->onFrameSet([this](ftl::rgbd::FrameSet &fs) -> bool {
		return post(fs);
	});
}

/*void Reconstruction::addSource(ftl::rgbd::Source *src) {
	//src->setChannel(Channel::Depth);
	group_->addSource(src); // TODO: check if source is already in group?
}

void Reconstruction::addRawCallback(const std::function<void(ftl::rgbd::Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &cb) {
	group_->addRawCallback(cb);
}*/

bool Reconstruction::render(ftl::rgbd::VirtualSource *vs, ftl::rgbd::Frame &out) {
	{
		UNIQUE_LOCK(exchange_mtx_, lk);
		if (new_frame_) {
			new_frame_ = false;
			fs_align_.swapTo(fs_render_);
		}
	}
	/*if (fs_render_.stale || fs_render_.timestamp <= 0) {
		LOG(ERROR) << "STALE FRAME TO RENDER";
		return false;
	}
	fs_render_.stale = true;*/

	// Create scene transform, intended for axis aligning the walls and floor
	Eigen::Matrix4d transform;
	//if (getConfig()["transform"].is_object()) {
		//auto &c = getConfig()["transform"];
		float rx = value("transform_pitch", 0.0f);
		float ry = value("transform_yaw", 0.0f);
		float rz = value("transform_roll", 0.0f);
		float x = value("transform_x", 0.0f);
		float y = value("transform_y", 0.0f);
		float z = value("transform_z", 0.0f);

		Eigen::Affine3d r = create_rotation_matrix(rx, ry, rz);
		Eigen::Translation3d trans(Eigen::Vector3d(x,y,z));
		Eigen::Affine3d t(trans);
		transform = t.matrix() * r.matrix();
		//LOG(INFO) << "Set transform: " << transform;
	//} else {
	//	transform.setIdentity();
	//}

	Eigen::Affine3d sm = Eigen::Affine3d(Eigen::Scaling(double(value("scale", 1.0f))));
	bool res = false; //renderer_->render(vs, out, sm.matrix() * transform);
	//fs_render_.resetFull();
	return res;
}