#include "observer_impl.hpp"
#include "frame_impl.hpp"
#include <voltu/types/errors.hpp>

#include <ftl/render/CUDARender.hpp>
#include <ftl/rgbd/frame.hpp>

using voltu::internal::ObserverImpl;
using ftl::rgbd::Capability;

ObserverImpl::ObserverImpl(ftl::Configurable *base)
 : id_(254)  // FIXME: Allocate this
{
	pool_ = new ftl::data::Pool(2,5);
	rend_ = ftl::create<ftl::render::CUDARender>(base, "camN");  // FIXME: Generate name properly

	intrinsics_.fx = 700.0f;
	intrinsics_.fy = 700.0f;
	intrinsics_.width = 1280;
	intrinsics_.height = 720;
	intrinsics_.cx = -1280.0f/2.0f;
	intrinsics_.cy = -720.0f/2.0f;
	intrinsics_.minDepth = 0.1f;
	intrinsics_.maxDepth = 12.0f;
	pose_.setIdentity();

	calibration_uptodate_.clear();
}

ObserverImpl::~ObserverImpl()
{
	frameset_.reset();
	delete rend_;
	delete pool_;
}

void ObserverImpl::setResolution(uint32_t w, uint32_t h)
{
	if (w < 100 || w > 10000 || h < 100 || h > 10000)
	{
		throw voltu::exceptions::BadParameterValue();
	}
	intrinsics_.width = w;
	intrinsics_.height = h;
	intrinsics_.cx = -int(w)/2;
	intrinsics_.cy = -int(h)/2;
	calibration_uptodate_.clear();
}

void ObserverImpl::setFocalLength(uint32_t f)
{
	if (f < 100 || f > 10000)
	{
		throw voltu::exceptions::BadParameterValue();
	}
	intrinsics_.fx = f;
	intrinsics_.fy = f;
	calibration_uptodate_.clear();
}

void ObserverImpl::setStereo(bool)
{

}

bool ObserverImpl::waitCompletion(int timeout)
{
	if (frame_complete_) return true;

	frame_complete_ = true;

	try
	{
		rend_->render();
		rend_->end();
	}
	catch(const std::exception& e)
	{
		throw voltu::exceptions::InternalRenderError();
	}
	
	cudaSafeCall(cudaStreamSynchronize(frameset_->frames[0].stream()));
	return true;
}

void ObserverImpl::submit(const voltu::FramePtr& frame)
{
	if (frame_complete_)
	{
		frame_complete_ = false;
		frameset_ = _makeFrameSet();

		ftl::rgbd::Frame &rgbdframe = frameset_->frames[0].cast<ftl::rgbd::Frame>();

		if (!rgbdframe.has(ftl::codecs::Channel::Calibration) || calibration_uptodate_.test_and_set())
		{
			rgbdframe.setLeft() = intrinsics_;

			auto &cap = rgbdframe.create<std::unordered_set<Capability>>(ftl::codecs::Channel::Capabilities);
			cap.clear();
			cap.emplace(Capability::VIDEO);
			cap.emplace(Capability::MOVABLE);
			cap.emplace(Capability::ADJUSTABLE);
			cap.emplace(Capability::VIRTUAL);
			cap.emplace(Capability::LIVE);
			if (rend_->value("projection", 0) == int(ftl::rgbd::Projection::EQUIRECTANGULAR)) cap.emplace(Capability::EQUI_RECT);

			auto &meta = rgbdframe.create<std::map<std::string,std::string>>(ftl::codecs::Channel::MetaData);
			meta["name"] = std::string("Virtual Camera"); //host_->value("name", host_->getID());
			//meta["id"] = host_->getID();
			meta["uri"] = std::string("device:render");
			meta["device"] = std::string("CUDA Render");
		}
		//if (!rgbdframe.has(ftl::codecs::Channel::Pose))
		//{
			rgbdframe.setPose() = pose_.cast<double>();
		//}

		int width = rgbdframe.getLeft().width;
		int height = rgbdframe.getLeft().height;
		
		// FIXME: Create opengl buffers here and map to cuda?
		auto &colour = rgbdframe.create<cv::cuda::GpuMat>(ftl::codecs::Channel::Colour);
		colour.create(height, width, CV_8UC4);
		rgbdframe.create<cv::cuda::GpuMat>(ftl::codecs::Channel::Depth).create(height, width, CV_32F);
		rgbdframe.createTexture<float>(ftl::codecs::Channel::Depth);

		rend_->begin(rgbdframe, ftl::codecs::Channel::Colour);
	}

	auto *fimp = dynamic_cast<voltu::internal::FrameImpl*>(frame.get());
	if (!fimp)
	{
		throw voltu::exceptions::InvalidFrameObject();
	}

	const auto &sets = fimp->getInternalFrameSets();
	for (const auto &fs : sets)
	{
		Eigen::Matrix4d pose;
		pose.setIdentity();

		try
		{
			rend_->submit(fs.get(), ftl::codecs::Channels<0>(ftl::codecs::Channel::Colour), pose);
		}
		catch (...)
		{
			throw voltu::exceptions::InternalRenderError();
		}
	}
}

void ObserverImpl::setPose(const Eigen::Matrix4f &p)
{
	pose_ = p;
}

voltu::FramePtr ObserverImpl::getFrame()
{
	waitCompletion(0);
	auto f = std::make_shared<voltu::internal::FrameImpl>();
	f->pushFrameSet(frameset_);
	return f;
}

voltu::PropertyPtr ObserverImpl::property(voltu::ObserverProperty)
{
	throw voltu::exceptions::NotImplemented();
}

std::shared_ptr<ftl::data::FrameSet> ObserverImpl::_makeFrameSet()
{
	int64_t timestamp = ftl::timer::get_time();

	auto newf = std::make_shared<ftl::data::FrameSet>(pool_, ftl::data::FrameID(id_,255), timestamp);
	for (size_t i=0; i<size_; ++i) {
		newf->frames.push_back(std::move(pool_->allocate(ftl::data::FrameID(id_, i), timestamp)));
	}

	newf->mask = 0xFF;
	newf->clearFlags();
	newf->store();
	return newf;
}
