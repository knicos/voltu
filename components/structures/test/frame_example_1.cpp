#include "catch.hpp"

#include <ftl/data/new_frame.hpp>

using ftl::data::Session;
using ftl::data::Frame;
using ftl::codecs::Channel;
using ftl::data::ChangeType;
using ftl::data::StorageMode;

namespace ftl {
namespace streams {

/* Mock Feed class */
class Feed {
	public:
	Feed() : buffer0_(nullptr, &session_,0,0), buffer1_(nullptr, &session_,0,0) {
		flush_handle_ = session_.onFlush([this](Frame &f, Channel c) {
			// Loop changes back to buffer.
			// Normally transmitted somewhere first.
			//buffer_.swapChannel(c, f);
			ChangeType cc = f.getChangeType(c);
			buffer0_.createAnyChange(c, (cc == ChangeType::LOCAL) ? ChangeType::FOREIGN : ChangeType::COMPLETED) = f.getAnyMutable(c);
			return true;
		});
	}

	inline Frame &buffer() { return buffer0_; }
	inline Session &session() { return session_; }

	inline void fakeDispatch() {
		buffer0_.moveTo(buffer1_);
		buffer0_ = Frame(nullptr, &session_,0,0);

		// Save any persistent changes
		buffer1_.store();
		// Transmit any forwarding changes and prevent further changes
		buffer1_.flush();  // TODO: use special dispatched function

		// Call the onFrame handler.
		// Would be in another thread in real version of this class.
		frame_handler_.trigger(buffer1_);
	}

	inline ftl::Handle onFrame(const std::function<bool(Frame&)> &cb) {
		return frame_handler_.on(cb);
	}

	Frame createFrame(int id) {
		// TODO: Give it the id and a timestamp
		Frame f(nullptr, &session_,0,0);
		f.store();
		return f;
	}

	private:
	ftl::Handler<Frame&> frame_handler_;
	Session session_;
	Frame buffer0_;
	Frame buffer1_;
	ftl::Handle flush_handle_;
};

}
}

using ftl::streams::Feed;

/* Test class */
struct VideoFrame {
	int gpudata;
	int matdata;
};

TEST_CASE("ftl::data::Frame full non-owner example", "[example]") {
	// Register channels somewhere at startup
	ftl::data::make_channel<VideoFrame>(Channel::Colour, "colour", StorageMode::TRANSIENT);
	ftl::data::make_channel<VideoFrame>(Channel::Depth, "depth", StorageMode::TRANSIENT);
	ftl::data::make_channel<std::list<std::string>>(Channel::Messages, "messages", StorageMode::AGGREGATE);
	ftl::data::make_channel<float>(Channel::Pose, "pose", StorageMode::PERSISTENT);

	Feed feed;

	int i=0;
	int changed = 0;
	ftl::Handle myhandle;

	auto h = feed.onFrame([&i,&feed,&myhandle,&changed](Frame &f) {
		// First frame received
		if (i++ == 0 ) {
			// User of Frame makes changes or reads values from state
			REQUIRE( f.get<float>(Channel::Pose) == 6.0f );
			REQUIRE( f.get<VideoFrame>(Channel::Depth).gpudata == 1 );

			// Create a new frame for same source for some return state
			Frame nf = feed.createFrame(f.id);
			nf.create<std::list<std::string>>(Channel::Messages) = {"First Message"};
			nf.create<std::list<std::string>>(Channel::Messages) = {"Second Message"};
			nf.create<int>(Channel::Control) = 3456;
			//nf.set<float>(Channel::Pose) = 7.0f;

			// Listen for this `Control` change to be confirmed
			myhandle = nf.onChange(Channel::Control, [&changed](Frame &f, Channel c) {
				changed++;
				return false;  // Call once only
			});
			
			// Either by destruction or manually, final action is flush to send
			nf.flush();
		// Second frame received
		} else {
			REQUIRE( changed == 1 );  // Change notified before `onFrame`
			REQUIRE( f.get<float>(Channel::Pose) == 6.0f );
			REQUIRE( f.get<int>(Channel::Control) == 3456 );
			REQUIRE( (*f.get<std::list<std::string>>(Channel::Messages).begin()) == "First Message" );
		}
		return true;
	});

	// Generate some incoming changes from network
	// Usually this is done inside the Feed class...
	feed.buffer().createChange<VideoFrame>(Channel::Colour, ChangeType::FOREIGN).gpudata = 1;
	feed.buffer().createChange<VideoFrame>(Channel::Depth, ChangeType::COMPLETED).gpudata = 1;
	feed.buffer().createChange<float>(Channel::Pose, ChangeType::FOREIGN) = 6.0f;

	// Fake a frame being received on network or from file
	feed.fakeDispatch();
	// And dispatch the response frame also
	feed.fakeDispatch();

	REQUIRE( i == 2 );

	// For testing only...
	ftl::data::clearRegistry();
}
