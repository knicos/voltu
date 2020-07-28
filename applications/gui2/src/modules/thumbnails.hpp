#pragma once

#include "../module.hpp"
#include "../screen.hpp"

namespace ftl {
namespace gui2 {

/**
 * Controller for thumbnail view.
 */
class ThumbnailsController : public Module {
public:
	using Module::Module;
	virtual ~ThumbnailsController();

	virtual void init() override;
	virtual void activate();

	void show_thumbnails();
	void show_camera(ftl::data::FrameID id);

	std::vector<ftl::data::FrameSetPtr> getFrameSets();

	void removeFrameset(uint32_t id);

private:
	std::mutex mtx_;
	std::map<unsigned int, ftl::data::FrameSetPtr> framesets_;
};

}
}
