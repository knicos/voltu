#pragma once

#include <voltu/types/frame.hpp>
#include <voltu/types/image.hpp>
#include <voltu/types/channel.hpp>
#include <ftl/data/new_frameset.hpp>

namespace voltu
{
namespace internal
{

class FrameImpl : public voltu::Frame
{
public:
	FrameImpl();
	~FrameImpl() override;

	std::list<voltu::ImagePtr> getImageSet(voltu::Channel) override;

	voltu::PointCloudPtr getPointCloud(voltu::PointCloudFormat cloudfmt, voltu::PointFormat pointfmt) override;

	std::vector<std::string> getMessages() override;

	int64_t getTimestamp() override;

	void pushFrameSet(const std::shared_ptr<ftl::data::FrameSet> &fs);

	inline const std::list<std::shared_ptr<ftl::data::FrameSet>> &getInternalFrameSets() const { return framesets_; }

private:
	std::list<std::shared_ptr<ftl::data::FrameSet>> framesets_;
};

}
}
