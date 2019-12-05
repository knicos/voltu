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

using ftl::Reconstruction;
using ftl::codecs::Channel;

Reconstruction::Reconstruction(nlohmann::json &config, const std::string name) :
	ftl::Configurable(config), busy_(false), fs_render_(), fs_align_() {
	group_ = new ftl::rgbd::Group;
	group_->setName("ReconGroup-" + name);
	group_->setLatency(4);

	renderer_ = ftl::create<ftl::render::Triangular>(this, "renderer", &fs_render_);

	pipeline_ = ftl::config::create<ftl::operators::Graph>(this, "pre_filters");
	pipeline_->append<ftl::operators::ClipScene>("clipping")->set("enabled", false);
	pipeline_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	//pipeline_->append<ftl::operators::HFSmoother>("hfnoise");  // Remove high-frequency noise
	pipeline_->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	//pipeline_->append<ftl::operators::SmoothChannel>("smoothing");  // Generate a smoothing channel
	//pipeline_->append<ftl::operators::ScanFieldFill>("filling");  // Generate a smoothing channel
	pipeline_->append<ftl::operators::CrossSupport>("cross");
	pipeline_->append<ftl::operators::DiscontinuityMask>("discontinuity");
	pipeline_->append<ftl::operators::CrossSupport>("cross2")->set("discon_support", true);
	pipeline_->append<ftl::operators::CullDiscontinuity>("remove_discontinuity");
	//pipeline_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
	pipeline_->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);
	pipeline_->append<ftl::operators::MultiViewMLS>("mvmls");

	group_->sync([this](ftl::rgbd::FrameSet &fs) -> bool {
		// TODO: pause
		
		if (busy_) {
			LOG(INFO) << "Group frameset dropped: " << fs.timestamp;
			return true;
		}
		busy_ = true;

		// Swap the entire frameset to allow rapid return
		fs.swapTo(fs_align_);

		ftl::pool.push([this](int id) {
			UNIQUE_LOCK(fs_align_.mtx, lk);
			pipeline_->apply(fs_align_, fs_align_, 0);
			
			// TODO: To use second GPU, could do a download, swap, device change,
			// then upload to other device. Or some direct device-2-device copy.
			fs_align_.swapTo(fs_render_);

			LOG(INFO) << "Align complete... " << fs_align_.timestamp;
			busy_ = false;
		});
		return true;
	});
}

Reconstruction::~Reconstruction() {
	// TODO delete
}

void Reconstruction::addSource(ftl::rgbd::Source *src) {
	src->setChannel(Channel::Depth);
	group_->addSource(src); // TODO: check if source is already in group?
}

void Reconstruction::render(ftl::rgbd::VirtualSource *vs, ftl::rgbd::Frame &out) {
	renderer_->render(vs, out);
}