#ifndef _FTL_STREAM_FILESTREAM_HPP_
#define _FTL_STREAM_FILESTREAM_HPP_

#include <ftl/streams/stream.hpp>

namespace ftl {
namespace stream {

/**
 * Provide a packet stream to/from a file. If the file already exists it is
 * opened readonly, if not it is created write only. A mode to support both
 * reading and writing (to re code it) could be supported by using a temp file
 * for writing and swapping files when finished. It must be possible to control
 * streaming rate from the file.
 */
class File : public Stream {
	public:
	explicit File(nlohmann::json &config);
	File(nlohmann::json &config, std::ifstream *);
	File(nlohmann::json &config, std::ofstream *);
	~File();

	bool onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &) override;

	bool post(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &) override;

	bool begin() override { return begin(true); }
	bool begin(bool dorun);
	bool end() override;
	bool active() override;

	/**
	 * Automatically tick through the frames using a timer. Threads are used.
	 */
	bool run();

	/**
	 * Manually tick through the frames one per call.
	 */
	bool tick(int64_t);

	/**
	 * Directly read a packet. Returns false if no more packets exist, true
	 * otherwise. The callback is called when a packet is read.
	 */
	bool readPacket(std::tuple<ftl::codecs::StreamPacket,ftl::codecs::Packet> &);

	enum class Mode {
		Read,
		Write,
		ReadWrite
	};

	inline void setMode(Mode m) { mode_ = m; }
	inline void setStart(int64_t ts) { timestamp_ = ts; }

	private:
	std::ofstream *ostream_;
	std::ifstream *istream_;

	bool checked_;
	Mode mode_;
	msgpack::sbuffer buffer_out_;
	msgpack::unpacker buffer_in_;
	std::list<std::tuple<ftl::codecs::StreamPacket,ftl::codecs::Packet>> data_;
	int64_t timestart_;
	int64_t timestamp_;
	int64_t interval_;
	int64_t first_ts_;
	bool active_;
	int version_;
	ftl::timer::TimerHandle timer_;

	StreamCallback cb_;
	MUTEX mutex_;
	MUTEX data_mutex_;
	std::atomic<int> jobs_;

	bool _open();
	bool _checkFile();
};

}
}

#endif  // _FTL_STREAM_FILESTREAM_HPP_
