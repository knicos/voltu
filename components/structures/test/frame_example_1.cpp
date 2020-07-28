#include "catch.hpp"

#include <ftl/codecs/packet.hpp>
#include <ftl/data/new_frame.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/timer.hpp>

#include <loguru.hpp>

using ftl::data::Session;
using ftl::data::Frame;
using ftl::codecs::Channel;
using ftl::data::ChangeType;
using ftl::data::StorageMode;
using ftl::data::FrameID;

namespace ftl {
namespace streams {

/* Mock Feed class */
class Feed {
	public:
	Feed() : pool_(5,10), buffer0_(std::move(pool_.allocate(FrameID(0,0),0))), buffer1_(std::move(pool_.allocate(FrameID(0,0),0))) {
		flush_handle_ = pool_.session(FrameID(0,0)).onFlush([this](Frame &f, Channel c) {
			// Loop changes back to buffer.
			// Normally transmitted somewhere first.
			// buffer1_.swapChannel(c, f);
			ChangeType cc = f.getChangeType(c);
			if (cc == ChangeType::RESPONSE) {
				ftl::codecs::Packet pkt;
				pkt.frame_count = 1;
				pkt.codec = ftl::codecs::codec_t::MSGPACK;
				pkt.bitrate = 255;
				pkt.flags = 0;
				
				auto encoder = ftl::data::getTypeEncoder(f.type(c));
				if (encoder) {
					if (encoder(f, c, pkt.data)) {
						buffer1_.informChange(c, ChangeType::FOREIGN, pkt);
					}
				} else {
					LOG(WARNING) << "Missing msgpack encoder";
				}
			} else if (cc == ChangeType::PRIMARY) {
				//ftl::codecs::Packet pkt(f.getEncoded(c).front());
				//buffer0_.informChange(c, (cc == ChangeType::PRIMARY) ? ChangeType::FOREIGN : ChangeType::COMPLETED, ???);
				buffer0_.swapChannel(c, f);
			}
			return true;
		});
	}

	inline Frame &buffer() { return buffer0_; }

	inline void fakeDispatch() {
		Frame f = std::move(buffer0_);
		buffer0_ = pool_.allocate(FrameID(0,0),ftl::timer::get_time());

		// Save any persistent changes
		f.store();
		// Transmit any forwarding changes and prevent further changes
		//f.flush();  // TODO: use special dispatched function

		// Call the onFrame handler.
		// Would be in another thread in real version of this class.
		frame_handler_.trigger(f);
	}

	inline Frame getFrame() {
		Frame f = std::move(buffer1_);
		buffer1_ = pool_.allocate(FrameID(0,0),ftl::timer::get_time());

		// Save any persistent changes
		f.store();
		return f;
	}

	inline ftl::Handle onFrame(const std::function<bool(Frame&)> &cb) {
		return frame_handler_.on(cb);
	}

	private:
	ftl::data::Pool pool_;
	ftl::Handler<Frame&> frame_handler_;
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

// Disable msgpack
template <>
inline bool ftl::data::make_type<VideoFrame>() {
	return false;
}

template <>
inline bool ftl::data::decode_type<VideoFrame>(std::any &a, const std::vector<uint8_t> &data) {
	return false;
}

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
		i++;

		// First frame received
		// User of Frame makes changes or reads values from state
		REQUIRE( f.get<float>(Channel::Pose) == 6.0f );
		REQUIRE( f.get<VideoFrame>(Channel::Depth).gpudata == 1 );

		// Create a new frame for same source for some return state
		Frame nf = f.response();
		nf.create<std::list<std::string>>(Channel::Messages) = "First Message";
		nf.create<std::list<std::string>>(Channel::Messages) = "Second Message";
		nf.create<int>(Channel::Control) = 3456;
		//nf.set<float>(Channel::Pose) = 7.0f;

		// Listen for this `Control` change to be confirmed
		myhandle = nf.onChange(Channel::Control, [&changed](Frame &f, Channel c) {
			changed++;
			return false;  // Call once only
		});

		// Either by destruction or manually, final action is flush to send
		nf.flush();

		return true;
	});

	// Generate some incoming changes from network
	// Usually this is done inside the Feed class...
	feed.buffer().createChange<VideoFrame>(Channel::Colour, ChangeType::FOREIGN).gpudata = 1;
	feed.buffer().createChange<VideoFrame>(Channel::Depth, ChangeType::COMPLETED).gpudata = 1;
	feed.buffer().createChange<float>(Channel::Pose, ChangeType::FOREIGN) = 6.0f;

	// Fake a frame being completely received on network or from file
	feed.fakeDispatch();

	// Now pretend to be an owner and create a new frame... it should have the
	// response data in it, so check for that.
	{
		Frame f = feed.getFrame();
		REQUIRE( changed == 1 );  // Change notified before `onFrame`
		REQUIRE( f.get<float>(Channel::Pose) == 6.0f );
		REQUIRE( f.get<int>(Channel::Control) == 3456 );
		REQUIRE( (*f.get<std::list<std::string>>(Channel::Messages).begin()) == "First Message" );
	}
	// We wont bother dispatching this new frame
	//feed.fakeDispatch();

	REQUIRE( i == 1 );

	// For testing only...
	ftl::data::clearRegistry();
}

TEST_CASE("ftl::data::Frame full owner example", "[example]") {
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
			Frame nf = f.response();
			nf.create<std::list<std::string>>(Channel::Messages) = "First Message";
			nf.create<std::list<std::string>>(Channel::Messages) = "Second Message";
			nf.create<int>(Channel::Control) = 3456;
			nf.set<float>(Channel::Pose) = 7.0f;

			// Listen for this `Control` change to be confirmed
			myhandle = nf.onChange(Channel::Control, [&changed](Frame &f, Channel c) {
				changed++;
				return false;  // Call once only
			});

			// Either by destruction or manually, final action is flush to send
			nf.flush();
		// Second frame received
		} else {

		}
		return true;
	});

	// Create an entirely new frame, destruction will send it.
	{
		Frame f = feed.getFrame();
		f.create<VideoFrame>(Channel::Colour).gpudata = 1;
		f.create<VideoFrame>(Channel::Depth).gpudata = 1;
		f.create<float>(Channel::Pose) = 6.0f;
	}
	// Trigger local onFrame callback with the above frame.
	feed.fakeDispatch();

	// Create next new frame, now includes response changes
	{
		Frame f = feed.getFrame();
		REQUIRE( changed == 1 );  // Change notified before `onFrame`
		REQUIRE( f.get<float>(Channel::Pose) == 7.0f );
		REQUIRE( f.get<int>(Channel::Control) == 3456 );
		REQUIRE( (*f.get<std::list<std::string>>(Channel::Messages).begin()) == "First Message" );
	}
	feed.fakeDispatch();

	REQUIRE( i == 2 );

	// For testing only...
	ftl::data::clearRegistry();
}
