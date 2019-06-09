#ifndef _FTL_RGBD_DISPLAY_HPP_
#define _FTL_RGBD_DISPLAY_HPP_

#include <nlohmann/json.hpp>
#include <ftl/rgbd/source.hpp>

using MouseAction = std::function<void(int, int, int, int)>;

namespace ftl {
namespace rgbd {

class Display : public ftl::Configurable {
	public:
	explicit Display(nlohmann::json &);
	Display(nlohmann::json &, Source *);
	~Display();

	void setSource(Source *src) { source_ = src; }
	void update();

	bool active() const { return active_; }

	void onKey(const std::function<void(int)> &h) { key_handlers_.push_back(h); }

	void wait(int ms);

	private:
	Source *source_;
	std::string name_;
	std::vector<std::function<void(int)>> key_handlers_;
	Eigen::Vector3f eye_;
	Eigen::Vector3f centre_;
	Eigen::Vector3f up_;
	Eigen::Vector3f lookPoint_;
	float lerpSpeed_;
	bool active_;
	MouseAction mouseaction_;

	static int viewcount__;

	void init();
};

}
}

#endif  // _FTL_RGBD_DISPLAY_HPP_
