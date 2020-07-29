#ifndef _FTL_STREAM_FEED_HPP_
#define _FTL_STREAM_FEED_HPP_

#include <ftl/configurable.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/handle.hpp>

#include <ftl/operators/operator.hpp>

#include <ftl/rgbd/source.hpp>
#include <ftl/data/framepool.hpp>

#include <ftl/streams/stream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/data/new_frameset.hpp>

#include <ftl/render/CUDARender.hpp>

namespace ftl {
namespace render { class Source; }
namespace stream {

struct SourceInfo {
	std::string uri;
	int64_t last_used;

	inline bool operator<(const SourceInfo &o) const { return last_used >= o.last_used; }
};

class Feed : public ftl::Configurable {
public:
	/**
	 * "Filtered feed"
	 */
	class Filter {
		friend Feed;
	public:
		const std::unordered_set<ftl::codecs::Channel>& channels() const { return channels_; };
		const std::unordered_set<ftl::codecs::Channel>& availableChannels() const { return channels_available_; };
		const std::unordered_set<uint32_t>& sources() const { return sources_; };

		Filter &select(const std::unordered_set<ftl::codecs::Channel> &cs);

		void on(const ftl::data::FrameSetCallback &cb);

		ftl::Handle onWithHandle(const ftl::data::FrameSetCallback &cb);

		void onRemove(const std::function<bool(uint32_t)> &cb);

		/**
		 * Safely obtain frameset pointers for all framesets matched by this
		 * filter. This will be the most recently seen frameset at the time of
		 * this call. Used by renderer, for example.
		 */
		std::list<ftl::data::FrameSetPtr> getLatestFrameSets();

		/** remove filter; any references/pointers become invalid */
		void remove();

	protected:
		/** removes filters, releases framesets */
		~Filter();

	private:
		Filter(Feed* feed, const std::unordered_set<uint32_t>& sources,  const std::unordered_set<ftl::codecs::Channel>& channels);
		Feed* feed_;
		std::unordered_set<ftl::codecs::Channel> channels_;
		std::unordered_set<ftl::codecs::Channel> channels_available_;
		std::unordered_set<uint32_t> sources_;
		ftl::Handler<const ftl::data::FrameSetPtr&> handler_;
		ftl::Handler<uint32_t> remove_handler_;
		std::vector<ftl::Handle> handles_;
	};

public:

	Feed(nlohmann::json &config, ftl::net::Universe *net);
	~Feed();

	/** list possible channels
	 * BUG:/TODO: only returns requested + persistent
	 */
	const std::unordered_set<ftl::codecs::Channel> availableChannels(ftl::data::FrameID);

	/** Add source (file path, device path or URI) */
	uint32_t add(const std::string &str);
	uint32_t add(const ftl::URI &uri);
	uint32_t getID(const std::string &source);
	std::string getURI(uint32_t fsid);

	/** Get current frameset (cached) */
	ftl::data::FrameSetPtr getFrameSet(uint32_t);

	/**
	 * Get the configurable ID that corresponds to the original source. For
	 * net stream sources this may be a remote configurable.
	 */
	std::string getSourceURI(ftl::data::FrameID id);

	void remove(const std::string &str);
	void remove(uint32_t id);

	std::vector<std::string> listSources();
	std::vector<ftl::data::FrameID> listFrames();
	std::vector<unsigned int> listFrameSets();

	std::set<ftl::stream::SourceInfo> recentSources();
	std::vector<std::string> knownHosts();
	std::vector<std::string> availableNetworkSources();
	std::vector<std::string> availableFileSources();
	std::vector<std::string> availableDeviceSources();
	std::vector<std::string> availableGroups();
	bool sourceAvailable(const std::string &uri);
	bool sourceActive(const std::string &uri);

	void clearFileHistory();
	void clearHostHistory();

	/**
	 * Perform a render tick for all render sources. Note that this must be
	 * called from the GUI / OpenGL thread.
	 */
	void render();

	void startRecording(Filter *, const std::string &filename);
	void startStreaming(Filter *, const std::string &filename);
	void startStreaming(Filter *);
	void stopRecording();
	bool isRecording();

	inline ftl::Handle onNewSources(const std::function<bool(const std::vector<std::string> &)> &cb) { return new_sources_cb_.on(cb); }

	inline ftl::Handle onAdd(const std::function<bool(uint32_t)> &cb) { return add_src_cb_.on(cb); }

	inline ftl::Handle onRemoveSources(const std::function<bool(uint32_t)> &cb) { return remove_sources_cb_.on(cb); }

	cv::Mat getThumbnail(const std::string &uri);
	std::string getName(const std::string &uri);

	void setPipelineCreator(const std::function<void(ftl::operators::Graph*)> &cb);

	ftl::operators::Graph* addPipeline(const std::string &name);
	ftl::operators::Graph* addPipeline(uint32_t fsid);
	/** Returns pointer to filter object. Pointers will be invalid after Feed
	 * is destroyed. User is required to call Filter::remove() when filter
	 * no longer required */
	Filter* filter(const std::unordered_set<uint32_t> &framesets, const std::unordered_set<ftl::codecs::Channel> &channels);
	Filter* filter(const std::unordered_set<std::string> &sources, const std::unordered_set<ftl::codecs::Channel> &channels);
	/** all framesets, selected channels */
	Filter* filter(const std::unordered_set<ftl::codecs::Channel> &channels);

	void removeFilter(Filter* filter);

	void autoConnect();

	void lowLatencyMode();

private:
	// public methods acquire lock if necessary, private methods assume locking
	// managed by caller
	SHARED_MUTEX mtx_;
	std::condition_variable cv_net_connect_;

	ftl::net::Universe* const net_;
	std::unique_ptr<ftl::data::Pool> pool_;
	std::unique_ptr<ftl::stream::Intercept> interceptor_;
	 // multiple streams to single fs
	std::unique_ptr<ftl::stream::Muxer> stream_;

	 // streams to fs
	std::unique_ptr<ftl::stream::Receiver> receiver_;
	ftl::Handle handle_receiver_;

	// framesets to stream
	std::unique_ptr<ftl::stream::Sender> sender_;
	ftl::Handle handle_sender_;

	std::unique_ptr<ftl::stream::Sender> recorder_;
	std::unique_ptr<ftl::stream::Broadcast> record_stream_;
	ftl::Handle handle_record_;
	ftl::Handle handle_record2_;
	ftl::Handle record_recv_handle_;
	ftl::Handle record_new_client_;
	Filter *record_filter_;

	//ftl::Handler<const ftl::data::FrameSetPtr&> frameset_cb_;
	std::unordered_map<std::string, uint32_t> fsid_lookup_;
	std::map<uint32_t, ftl::data::FrameSetPtr> latest_;
	std::unordered_map<std::string, uint32_t> groups_;

	std::unordered_map<uint32_t, std::list<ftl::stream::Stream*>> streams_;
	std::unordered_map<uint32_t, ftl::rgbd::Source*> devices_;
	std::unordered_map<uint32_t, ftl::render::Source*> renderers_;
	std::unordered_map<uint32_t, ftl::operators::Graph*> pre_pipelines_;
	std::list<ftl::streams::ManualSourceBuilder*> render_builders_;
	std::function<void(ftl::operators::Graph*)> pipe_creator_;

	std::unordered_set<std::string> netcams_;
	ftl::Handler<const std::vector<std::string> &> new_sources_cb_;
	ftl::Handler<uint32_t> add_src_cb_;
	ftl::Handler<uint32_t> remove_sources_cb_;

	std::vector<Filter*> filters_;

	uint32_t fs_counter_ = 0;
	uint32_t file_counter_ = 0;

	uint32_t allocateFrameSetId(const std::string &group);

	void add(uint32_t fsid, const std::string &uri, ftl::stream::Stream *s);
	nlohmann::json &_add_recent_source(const ftl::URI &uri);
	void _saveThumbnail(const ftl::data::FrameSetPtr& fs);

	/** callback for network (adds new sorces on connect/...) */
	void _updateNetSources(ftl::net::Peer *p, bool autoadd=false);
	void _updateNetSources(ftl::net::Peer *p, const std::string &uri, bool autoadd=false);
	/** select channels and sources based on current filters_; */
	void select();

	void _createPipeline(uint32_t fsid);
	ftl::operators::Graph* _addPipeline(uint32_t fsid);

	void _beginRecord(Filter *f);
	void _stopRecording();
	bool _isRecording();
};

}
}

#endif
