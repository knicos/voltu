#pragma once

#include <memory>
#include <mutex>
#include <array>

#include <ftl/handle.hpp>
#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/master.hpp>

#include <ftl/streams/stream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/feed.hpp>

#include <ftl/streams/filestream.hpp>
#include <ftl/audio/speaker.hpp>

#include <ftl/data/new_frame.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/data/framepool.hpp>


namespace ftl {
namespace gui2 {

class InputOutput {
public:
	InputOutput(ftl::Configurable *config, ftl::net::Universe *net);
	InputOutput(const InputOutput&) = delete;
	void operator=(const InputOutput&) = delete;

	ftl::Handle addCallback(const std::function<bool(const ftl::data::FrameSetPtr&)>&);

	ftl::net::Universe* net() const;
	ftl::ctrl::Master* master() const { return master_.get(); }
	ftl::stream::Feed* feed() const { return feed_.get(); }
	ftl::audio::Speaker* speaker() const { return speaker_; }

private:
	ftl::net::Universe* net_;
	std::unique_ptr<ftl::stream::Feed> feed_;
	std::unique_ptr<ftl::ctrl::Master> master_;
	ftl::audio::Speaker *speaker_;


};

}
}
