#ifndef _FTL_STREAMS_FEED_HPP_
#define _FTL_STREAMS_FEED_HPP_

#include <ftl/configurable.hpp>

namespace ftl {
namespace streams {

using FrameSetHandler = std::function<bool(ftl::data::FrameSet&)>;

class Feed : public ftl::Configurable {
	public:

	void addStream(ftl::streams::Stream *s);

	ftl::Handler onFrameSet(const FrameSetHandler &fsh);

	uint32_t allocateFrameId();

	void createFrame(uint32_t id, ftl::data::Frame &f);

	void createFrameSet(uint32_t id, int size, ftl::data::FrameSet &fs);

	private:
	ftl::streams::Receiver *receiver_;
	ftl::streams::Sender *sender_;
	std::unordered_map<uint32_t, ftl::data::Session> stores_;
};

}
}

#endif