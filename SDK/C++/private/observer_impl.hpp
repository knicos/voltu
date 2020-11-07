#pragma once

#include <voltu/observer.hpp>
#include <ftl/render/renderer.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/rgbd/camera.hpp>
#include <Eigen/Eigen>

namespace voltu
{
namespace internal
{

class ObserverImpl : public voltu::Observer
{
public:
	explicit ObserverImpl(ftl::Configurable *base);

	~ObserverImpl() override;

	void setResolution(uint32_t w, uint32_t h) override;

	void setFocalLength(uint32_t f) override;

	void setStereo(bool) override;

	bool waitCompletion(int timeout) override;

	void submit(const voltu::FramePtr&) override;

	void setPose(const Eigen::Matrix4f &) override;

	voltu::FramePtr getFrame() override;

	voltu::PropertyPtr property(voltu::ObserverProperty) override;

private:
	ftl::render::FSRenderer *rend_;
	ftl::data::Pool *pool_;
	int id_;
	size_t size_ = 1;
	std::shared_ptr<ftl::data::FrameSet> frameset_;
	bool frame_complete_ = true;
	ftl::rgbd::Camera intrinsics_;
	Eigen::Matrix4f pose_;
	std::atomic_flag calibration_uptodate_;

	std::shared_ptr<ftl::data::FrameSet> _makeFrameSet();
};

}
}
