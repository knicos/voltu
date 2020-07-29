#include <loguru.hpp>
#include <nlohmann/json.hpp>
#include <ftl/streams/feed.hpp>
#include <ftl/streams/renderer.hpp>

#include <ftl/streams/netstream.hpp>
#include <ftl/streams/filestream.hpp>

#include "ftl/operators/colours.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/antialiasing.hpp"
#include <ftl/operators/smoothing.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/detectandtrack.hpp>
#include <ftl/operators/weighting.hpp>
#include <ftl/operators/mvmls.hpp>
#include <ftl/operators/clipping.hpp>
#include <ftl/operators/poser.hpp>
#include <ftl/operators/gt_analysis.hpp>

using ftl::stream::Feed;
using ftl::codecs::Channel;

//static nlohmann::json feed_config;

////////////////////////////////////////////////////////////////////////////////

Feed::Filter::Filter(Feed* feed, const std::unordered_set<uint32_t>& sources, const std::unordered_set<Channel>& channels) :
		feed_(feed), channels_(channels), channels_available_(channels), sources_(sources) {

};

Feed::Filter::~Filter() {

}

void Feed::Filter::remove() {
	return feed_->removeFilter(this);
}

void Feed::Filter::on(const ftl::data::FrameSetCallback &cb) {
	UNIQUE_LOCK(feed_->mtx_, lk);

	if (std::find(feed_->filters_.begin(), feed_->filters_.end(),this) == feed_->filters_.end()) {
		throw ftl::exception("Filter does not belong to Feed; This should never happen!");
	}

	handles_.push_back(std::move(handler_.on(cb)));
}

ftl::Handle Feed::Filter::onWithHandle(const ftl::data::FrameSetCallback &cb) {
	UNIQUE_LOCK(feed_->mtx_, lk);

	if (std::find(feed_->filters_.begin(), feed_->filters_.end(),this) == feed_->filters_.end()) {
		throw ftl::exception("Filter does not belong to Feed; This should never happen!");
	}

	return std::move(handler_.on(cb));
}

std::list<ftl::data::FrameSetPtr> Feed::Filter::getLatestFrameSets() {
	std::list<ftl::data::FrameSetPtr> results;

	SHARED_LOCK(feed_->mtx_, lk);
	if (sources_.empty()) {
		for (auto &i : feed_->latest_) {
			if (i.second) results.emplace_back(std::atomic_load(&(i.second)));
		}
	} else {
		for (auto &s : sources_) {
			auto i = feed_->latest_.find(s);
			if (i != feed_->latest_.end()) {
				if (i->second) results.emplace_back(std::atomic_load(&(i->second)));
			}
		}
	}
	return results;
}

Feed::Filter &Feed::Filter::select(const std::unordered_set<ftl::codecs::Channel> &cs) {
	UNIQUE_LOCK(feed_->mtx_, lk);
	channels_ = cs;
	feed_->select();
	return *this;
}

////////////////////////////////////////////////////////////////////////////////

Feed::Feed(nlohmann::json &config, ftl::net::Universe*net) :
		ftl::Configurable(config), net_(net) {

	//feed_config = ftl::loadJSON(FTL_LOCAL_CONFIG_ROOT "/feed.json");
	restore(ftl::Configurable::getID(), {
		"recent_files",
		"recent_sources",
		"known_hosts",
		"known_groups",
		"auto_host_connect",
		"auto_host_sources",
		"uri",
		"recorder"
	});

	pool_ = std::make_unique<ftl::data::Pool>(3,5);

	stream_ = std::unique_ptr<ftl::stream::Muxer>
		(ftl::create<ftl::stream::Muxer>(this, "muxer"));

	interceptor_ = std::unique_ptr<ftl::stream::Intercept>
		(ftl::create<ftl::stream::Intercept>(this, "intercept"));

	receiver_ = std::unique_ptr<ftl::stream::Receiver>
		(ftl::create<ftl::stream::Receiver>(this, "receiver", pool_.get()));

	sender_ = std::unique_ptr<ftl::stream::Sender>
		(ftl::create<ftl::stream::Sender>(this, "sender"));

	recorder_ = std::unique_ptr<ftl::stream::Sender>
		(ftl::create<ftl::stream::Sender>(this, "recorder"));
	record_stream_ = std::unique_ptr<ftl::stream::Broadcast>
		(ftl::create<ftl::stream::Broadcast>(this, "record_stream"));
	recorder_->setStream(record_stream_.get());

	record_recv_handle_ = record_stream_->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		receiver_->processPackets(spkt, pkt);
		return true;
	});

	record_filter_ = nullptr;

	//interceptor_->setStream(stream_.get());
	receiver_->setStream(stream_.get());
	sender_->setStream(stream_.get());

	handle_sender_ = pool_->onFlush([this]
			(ftl::data::Frame &f, ftl::codecs::Channel c) {

		// Send only reponse channels on a per frame basis
		if (f.mode() == ftl::data::FrameMode::RESPONSE) {
			// Remote sources need to use sender, otherwise loopback to local
			if (streams_.find(f.frameset()) != streams_.end()) {
				sender_->post(f, c);
			} else {
				receiver_->loopback(f, c);
			}
		}
		return true;
	});

	net_->onConnect([this](ftl::net::Peer *p) {
		ftl::pool.push([this,p](int id) {
			_updateNetSources(p);
		});
	});

	if (net_->isBound("add_stream")) net_->unbind("add_stream");
	net_->bind("add_stream", [this](ftl::net::Peer &p, std::string uri){
		//UNIQUE_LOCK(mtx_, lk);
		_updateNetSources(&p, uri);
	});

	net_->onDisconnect([this](ftl::net::Peer *) {
		// TODO: maintain map between peer and sources, on disconnect remove all
		//       peer's source. Also map between Peers and fsids?
		//std::unique_lock<std::mutex> lk(mtx_);
	});

	handle_receiver_ = receiver_->onFrameSet(
		[this](const ftl::data::FrameSetPtr& fs) {
			if (value("drop_partial_framesets", false)) {
				if (fs->count < static_cast<int>(fs->frames.size())) {
					LOG(WARNING) << "Dropping partial frameset: " << fs->timestamp();
					return true;
				}
			}

			// FIXME: What happens if pipeline added concurrently?
			if (pre_pipelines_.count(fs->frameset()) == 1) {
				pre_pipelines_[fs->frameset()]->apply(*fs, *fs, 0);
			}

			SHARED_LOCK(mtx_, lk);

			std::atomic_store(&latest_.at(fs->frameset()), fs);

			if (fs->hasAnyChanged(Channel::Thumbnail)) {
				_saveThumbnail(fs);
			}

			for (auto* filter : filters_) {
				// TODO: smarter update (update only when changed) instead of
				// filter->channels_available_ = fs->channels();

				if (filter->sources().empty()) {
					//filter->channels_available_ = fs->channels();
					filter->handler_.trigger(fs);
				}
				else {
					// TODO: process partial/complete sets here (drop), that is
					//       intersection filter->sources() and fs->sources() is
					//       same as filter->sources().

					// TODO: reverse map source ids required here?
					for (const auto& src : filter->sources()) {
						//if (fs->hasFrame(src)) {
						if (fs->frameset() == src) {
							//filter->channels_available_ = fs->channels();
							filter->handler_.trigger(fs);
							break;
						}
					}
				}
			}

			return true;
	});

	stream_->begin();

	//if (value("auto_host_connect", true)) autoConnect();
}

Feed::~Feed() {
	UNIQUE_LOCK(mtx_, lk);
	//ftl::saveJSON(FTL_LOCAL_CONFIG_ROOT "/feed.json", feed_config);

	handle_receiver_.cancel();
	handle_record_.cancel();
	handle_sender_.cancel();
	record_recv_handle_.cancel();

	receiver_.reset();  // Note: Force destruction first to remove filters this way
	sender_.reset();
	recorder_.reset();

	// TODO stop everything and clean up
	// delete

	for (auto &p : pre_pipelines_) {
		delete p.second;
	}
	for (auto &d : devices_) {
		delete d.second;
	}
	for (auto &r : renderers_) {
		lk.unlock();
		delete r.second;
		lk.lock();
	}

	if (filters_.size() > 0) LOG(WARNING) << "Filters remain after feed destruct (" << filters_.size() << ")";
	for (auto* filter : filters_) {
		delete filter;
	}
	filters_.clear();

	interceptor_.reset();
	stream_.reset();
	for (auto &ls : streams_) {
		for (auto *s : ls.second) {
			delete s;
		}
	}
}

void Feed::_saveThumbnail(const ftl::data::FrameSetPtr& fs) {
	// TODO: Put thumb somewhere here...
}

uint32_t Feed::allocateFrameSetId(const std::string &group) {
	if (group.size() == 0) {
		return fs_counter_++;
	} else {
		auto i = groups_.find(group);
		if (i == groups_.end()) {
			uint32_t id = fs_counter_++;
			groups_[group] = id;
			return id;
		} else {
			return i->second;
		}
	}
}

void Feed::select() {
	std::map<uint32_t, std::unordered_set<Channel>> selected_channels;
	for (auto &filter : filters_) {
		const auto& selected = filter->channels();

		if (filter->sources().empty()) {
			// no sources: select all sources with selected channels
			for (const auto& [uri, fsid] : fsid_lookup_) {
				std::ignore = uri;
				selected_channels[fsid].insert(selected.begin(), selected.end());
			}
		}
		else {
			// sources given
			for (const auto& fsid : filter->sources()) {
				if (selected_channels.count(fsid) == 0) {
					selected_channels.try_emplace(fsid);
				}
				selected_channels[fsid].insert(selected.begin(), selected.end());
			}
		}
	}
	for (auto& [fsid, channels] : selected_channels) {
		stream_->select(fsid, channels, true);
		LOG(INFO) << "Update selections";
		for (auto c : channels) {
			LOG(INFO) << "  -- select " << (int)c;
		}
	}
}

std::vector<std::string> Feed::listSources() {
	std::vector<std::string> sources;
	SHARED_LOCK(mtx_, lk);
	sources.reserve(fsid_lookup_.size());
	for (auto& [uri, fsid] : fsid_lookup_) {
		std::ignore = fsid;
		sources.push_back(uri);
	}
	return sources;
}

Feed::Filter* Feed::filter(const std::unordered_set<uint32_t> &framesets,
		const std::unordered_set<Channel> &channels) {

	auto* filter = new Filter(this, framesets, channels);
	UNIQUE_LOCK(mtx_, lk);
	filters_.push_back(filter);
	select();
	return filter;
}

Feed::Filter* Feed::filter(const std::unordered_set<Channel> &channels) {
	return filter(std::unordered_set<uint32_t>{}, channels);
}

Feed::Filter* Feed::filter(const std::unordered_set<std::string> &sources, const std::unordered_set<Channel> &channels) {
	std::unordered_set<uint32_t> fsids;

	SHARED_LOCK(mtx_, lk);
	for (const auto &src : sources) {
		ftl::URI uri(src);

		auto i = fsid_lookup_.find(uri.getBaseURI());
		if (i != fsid_lookup_.end()) {
			fsids.emplace(i->second);
		}
	}
	return filter(fsids, channels);
}

void Feed::remove(const std::string &str) {
	uint32_t fsid;

	{
		UNIQUE_LOCK(mtx_, lk);
		auto i = fsid_lookup_.find(str);
		if (i != fsid_lookup_.end()) {
			fsid = i->second;
		} else {
			return;
		}
	}

	remove(fsid);
}

void Feed::remove(uint32_t id) {
	UNIQUE_LOCK(mtx_, lk);

	// First tell all filters
	for (auto *f : filters_) {
		if (f->sources_.empty() || f->sources_.count(id)) {
			f->remove_handler_.trigger(id);
		}
	}

	remove_sources_cb_.trigger(id);

	// TODO: Actual delete of source
	// If stream source, remove from muxer
	if (streams_.count(id)) {
		auto &streams = streams_[id];
		for (auto *s : streams) {
			stream_->remove(s);
			delete s;
		}

		streams_.erase(id);
	} else if (devices_.count(id)) {
		receiver_->removeBuilder(id);
		delete devices_[id];
		devices_.erase(id);
	} else if (renderers_.count(id)) {

	}

	if (latest_.count(id)) latest_.erase(id);

	for (auto i = fsid_lookup_.begin(); i != fsid_lookup_.end();) {
		if (i->second == id) {
			i = fsid_lookup_.erase(i);
		} else {
			++i;
		}
	}
}

ftl::operators::Graph* Feed::addPipeline(uint32_t fsid) {
	UNIQUE_LOCK(mtx_, lk);
	return _addPipeline(fsid);
}

ftl::operators::Graph* Feed::_addPipeline(uint32_t fsid) {
	if (pre_pipelines_.count(fsid) != 0) {
		delete pre_pipelines_[fsid];
	}

	if (devices_.count(fsid)) {
		pre_pipelines_[fsid] = ftl::config::create<ftl::operators::Graph>
			(devices_[fsid], std::string("pipeline"));
	} else if (renderers_.count(fsid)) {
		pre_pipelines_[fsid] = ftl::config::create<ftl::operators::Graph>
			(renderers_[fsid], std::string("pipeline"));
	} else if (streams_.count(fsid)) {
		pre_pipelines_[fsid] = ftl::config::create<ftl::operators::Graph>
			(streams_[fsid].front(), std::string("pipeline"));
	}

	//pre_pipelines_[fsid] = ftl::config::create<ftl::operators::Graph>
	//	(this, std::string("pre_filters") + std::to_string(fsid));

	return pre_pipelines_[fsid];
}

void Feed::_createPipeline(uint32_t fsid) {
	// Don't recreate if already exists
	if (pre_pipelines_.count(fsid)) return;

	LOG(INFO) << "Creating pipeline";
	auto *p = _addPipeline(fsid);

	if (pipe_creator_) {
		pipe_creator_(p);
	} else {
		p->append<ftl::operators::DepthChannel>("depth")->value("enabled", false);
		p->append<ftl::operators::ClipScene>("clipping")->value("enabled", false);
		p->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
		p->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
		p->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
		//p->append<ftl::operators::HFSmoother>("hfnoise");
		p->append<ftl::operators::CrossSupport>("cross");
		p->append<ftl::operators::PixelWeights>("weights");
		p->append<ftl::operators::CullWeight>("remove_weights")->value("enabled", false);
		p->append<ftl::operators::DegradeWeight>("degrade");
		p->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);
		p->append<ftl::operators::BorderMask>("border_mask");
		p->append<ftl::operators::CullDiscontinuity>("remove_discontinuity");
		p->append<ftl::operators::MultiViewMLS>("mvmls")->value("enabled", false);
		p->append<ftl::operators::Poser>("poser")->value("enabled", true);
		p->append<ftl::operators::GTAnalysis>("gtanalyse");
	}
}

void Feed::setPipelineCreator(const std::function<void(ftl::operators::Graph*)> &cb) {
	UNIQUE_LOCK(mtx_, lk);
	pipe_creator_ = cb;
}

void Feed::removeFilter(Feed::Filter* filter) {
	UNIQUE_LOCK(mtx_, lk);

	if (record_filter_ == filter) {
		_stopRecording();
	}

	auto iter = std::find(filters_.begin(), filters_.end(), filter);
	if (iter != filters_.end()) {
		filters_.erase(iter);
		delete filter;
	}
}

void Feed::_updateNetSources(ftl::net::Peer *p, const std::string &s, bool autoadd) {
	UNIQUE_LOCK(mtx_, lk);
	netcams_.insert(s);

	// TODO: Auto add source

	ftl::URI uri(s);
	_add_recent_source(uri)["host"] = p->getURI();

	if (autoadd || value("auto_host_sources", false)) {
		add(uri);
	}

	ftl::pool.push([this, s](int id) {
		std::vector<std::string> srcs{s};
		new_sources_cb_.trigger(srcs);
	});
}

void Feed::_updateNetSources(ftl::net::Peer *p, bool autoadd) {
	//auto netcams =
	//	net_->findAll<std::string>("list_streams");

	// Peer may not have a list_streams binding yet
	try {
		auto peerstreams = p->call<std::vector<std::string>>("list_streams");


		UNIQUE_LOCK(mtx_, lk);
		//netcams_ = std::move(netcams);
		netcams_.insert(peerstreams.begin(), peerstreams.end());

		for (const auto &s : peerstreams) {
			ftl::URI uri(s);
			_add_recent_source(uri)["host"] = p->getURI();

			if (autoadd || value("auto_host_sources", false)) {
				ftl::pool.push([this, uri](int id) { add(uri); });
			}
		}

		ftl::pool.push([this, peerstreams](int id) {
			new_sources_cb_.trigger(peerstreams);
		});

	} catch (const ftl::exception &e) {

	}

	/* done by add()
	if (n > 0) {
		stream_->begin();
	}*/
}

std::vector<std::string> Feed::availableNetworkSources() {
	SHARED_LOCK(mtx_, lk);
	std::vector<std::string> result(netcams_.begin(), netcams_.end());
	return result;;
}

std::vector<std::string> Feed::availableGroups() {
	std::vector<std::string> groups;
	auto &known = getConfig()["known_groups"];

	for (auto &f : known.items()) {
		groups.push_back(f.key());
	}

	return groups;
}

std::vector<std::string> Feed::availableFileSources() {
	std::vector<std::string> files;
	auto &recent_files = getConfig()["recent_files"];

	for (auto &f : recent_files.items()) {
		files.push_back(f.key());
	}

	return files;
}

void Feed::clearFileHistory() {
	UNIQUE_LOCK(mtx_, lk);
	auto &recent_files = getConfig()["recent_files"];
	recent_files.clear();

	auto &recent = getConfig()["recent_sources"];
	for (auto i=recent.begin(); i != recent.end();) {
		ftl::URI uri(i.key());
		if (uri.getScheme() == ftl::URI::SCHEME_FILE) {
			i = recent.erase(i);
		} else {
			++i;
		}
	}
}

std::vector<std::string> Feed::knownHosts() {
	std::vector<std::string> hosts;
	auto &known = getConfig()["known_hosts"];

	for (auto &f : known.items()) {
		hosts.push_back(f.key());
	}

	return hosts;
}

void Feed::clearHostHistory() {
	UNIQUE_LOCK(mtx_, lk);
	auto &known = getConfig()["known_hosts"];
	known.clear();

	auto &recent = getConfig()["recent_sources"];
	for (auto i=recent.begin(); i != recent.end();) {
		ftl::URI uri(i.key());
		if (uri.getScheme() == ftl::URI::SCHEME_TCP || uri.getScheme() == ftl::URI::SCHEME_WS) {
			i = recent.erase(i);
		} else {
			++i;
		}
	}
}

std::set<ftl::stream::SourceInfo> Feed::recentSources() {
	std::set<ftl::stream::SourceInfo> result;

	auto &recent = getConfig()["recent_sources"];

	for (auto &f : recent.items()) {
		ftl::stream::SourceInfo info;
		info.uri = f.key();
		if (f.value().contains("uri")) info.uri = f.value()["uri"].get<std::string>();
		info.last_used = f.value()["last_open"].get<int64_t>();
		result.insert(info);
	}

	return result;
}

std::vector<std::string> Feed::availableDeviceSources() {
	std::vector<std::string> results;

	if (ftl::rgbd::Source::supports("device:pylon")) results.emplace_back("device:pylon");
	if (ftl::rgbd::Source::supports("device:camera")) results.emplace_back("device:camera");
	if (ftl::rgbd::Source::supports("device:stereo")) results.emplace_back("device:stereo");
	if (ftl::rgbd::Source::supports("device:screen")) results.emplace_back("device:screen");
	if (ftl::rgbd::Source::supports("device:realsense")) results.emplace_back("device:realsense");
	if (ftl::render::Source::supports("device:render")) results.emplace_back("device:render");
	if (ftl::render::Source::supports("device:openvr")) results.emplace_back("device:openvr");

	return results;
}

void Feed::autoConnect() {
	ftl::pool.push([this](int id) {
		auto &known_hosts = getConfig()["known_hosts"];

		for (auto &h : known_hosts.items()) {
			net_->connect(h.key())->noReconnect();
		}
	});
}

bool Feed::sourceAvailable(const std::string &uri) {
	return false;
}

bool Feed::sourceActive(const std::string &suri) {
	ftl::URI uri(suri);

	if (uri.getScheme() == ftl::URI::SCHEME_TCP || uri.getScheme() == ftl::URI::SCHEME_WS) {
		return net_->isConnected(uri);
	} else if (uri.getScheme() == ftl::URI::SCHEME_GROUP) {
		// Check that every constituent source is active
		auto &known = getConfig()["known_groups"];
		if (known.contains(uri.getBaseURI())) {
			auto &sources = known[uri.getBaseURI()]["sources"];

			for (auto i=sources.begin(); i!=sources.end(); ++i) {
				if (!sourceActive(i.key())) return false;
			}
		}
		return true;
	} else {
		SHARED_LOCK(mtx_, lk);
		return fsid_lookup_.count(uri.getBaseURI()) > 0;
	}
}

std::string Feed::getName(const std::string &puri) {
	ftl::URI uri(puri);

	if (uri.isValid() == false) return "Invalid";

	if (uri.getScheme() == ftl::URI::SCHEME_FTL) {
		if (uri.hasAttribute("name")) return uri.getAttribute<std::string>("name");
		try {
			auto *cfgble = ftl::config::find(puri);
			if (cfgble) {
				auto &j = cfgble->getConfig();
				std::string name = (j.is_structured()) ? j.value("name", j.value("uri", uri.getPathSegment(-1))) : uri.getPathSegment(-1);
				return (name.size() == 0) ? uri.getHost() : name;
			} else {
				std::string name = uri.getPathSegment(-1);
				return (name.size() == 0) ? uri.getHost() : name;
			}
			/*auto n = net_->findOne<std::string>("get_cfg", puri);
			if (n) {
				auto j = nlohmann::json::parse(*n);
				return (j.is_structured()) ? j.value("name", j.value("uri", uri.getPathSegment(-1))) : uri.getPathSegment(-1);
			}*/
		} catch (const ftl::exception &e) {
			e.ignore();
		}
		return puri;
	} else if (uri.getScheme() == ftl::URI::SCHEME_DEVICE) {
		if (uri.getPathSegment(0) == "pylon") return "Pylon";
		if (uri.getPathSegment(0) == "camera") return "Web Cam";
		if (uri.getPathSegment(0) == "stereo") return "Stereo";
		if (uri.getPathSegment(0) == "realsense") return "Realsense";
		if (uri.getPathSegment(0) == "screen") return "Screen Capture";
		if (uri.getPathSegment(0) == "render") return "3D Virtual";
		if (uri.getPathSegment(0) == "openvr") return "OpenVR";
		return "Unknown Device";
	} else if (uri.getScheme() == ftl::URI::SCHEME_FILE) {
		auto &recent_files = getConfig()["recent_files"];
		if (recent_files.is_structured() && recent_files.contains(uri.getBaseURI())) {
			return recent_files[uri.getBaseURI()].value("name", uri.getPathSegment(-1));
		} else {
			LOG(INFO) << "Missing file: " << puri;
			return uri.getPathSegment(-1);
		}
	} else if (uri.getScheme() == ftl::URI::SCHEME_TCP || uri.getScheme() == ftl::URI::SCHEME_WS) {
		return uri.getBaseURI();
	} else if (uri.getScheme() == ftl::URI::SCHEME_GROUP) {
		auto &groups = getConfig()["known_groups"];
		if (groups.contains(uri.getBaseURI())) {
			return uri.getPathSegment(0) + std::string(" (") + std::to_string(groups[uri.getBaseURI()]["sources"].size()) + std::string(")");
		} else {
			return uri.getPathSegment(0);
		}
	}

	return uri.getPathSegment(-1);
}

nlohmann::json &Feed::_add_recent_source(const ftl::URI &uri) {
	auto &known = getConfig()["recent_sources"];
	auto &details = known[uri.getBaseURI()];
	std::string name = uri.getPathSegment(-1);

	if (uri.hasAttribute("name")) {
		name = uri.getAttribute<std::string>("name");
	} else if (uri.getScheme() == ftl::URI::SCHEME_FILE) {
		name = name.substr(0, name.find_last_of('.'));
	}

	details["uri"] = uri.to_string();
	details["name"] = name;
	details["last_open"] = ftl::timer::get_time();

	if (uri.hasAttribute("group")) {
		std::string grpname = uri.getAttribute<std::string>("group");
		auto &groups = getConfig()["known_groups"];
		auto &grpdetail = groups[std::string("group:")+grpname];
		grpdetail["sources"][uri.getBaseURI()] = true;
	}

	return details;
}

void Feed::add(uint32_t fsid, const std::string &uri, ftl::stream::Stream* stream) {
	fsid_lookup_[uri] = fsid;
	latest_[fsid] = nullptr;
	streams_[fsid].push_back(stream);

	_createPipeline(fsid);

	stream_->add(stream, fsid);
	stream_->begin();
	stream_->select(fsid, {Channel::Colour}, true);
}

uint32_t Feed::add(const std::string &path) {
	ftl::URI uri(path);
	return add(uri);
}

uint32_t Feed::add(const ftl::URI &uri) {
	UNIQUE_LOCK(mtx_, lk);

	//if (!uri.isValid()) throw FTL_Error("Invalid URI: " << path);

	if (fsid_lookup_.count(uri.getBaseURI()) > 0) return fsid_lookup_[uri.getBaseURI()];

	const auto scheme = uri.getScheme();
	const std::string group = uri.getAttribute<std::string>("group");

	if ((scheme == ftl::URI::SCHEME_OTHER) || // assumes relative path
		(scheme == ftl::URI::SCHEME_FILE)) {

		auto eix = ((scheme == ftl::URI::SCHEME_OTHER) ? uri.getBaseURI() : uri.getPath()).find_last_of('.');
		auto ext = ((scheme == ftl::URI::SCHEME_OTHER) ? uri.getBaseURI() : uri.getPath()).substr(eix+1);

		if (ext != "ftl") {
			throw FTL_Error("Bad filename (expects .ftl) : " << uri.getBaseURI());
		}

		const int fsid = allocateFrameSetId(group);
		auto* fstream = ftl::create<ftl::stream::File>
			(this, std::string("ftlfile-") + std::to_string(file_counter_++));

		if (scheme == ftl::URI::SCHEME_OTHER) {
			fstream->set("filename", uri.getBaseURI());
		}
		else {
			// possible BUG: uri.getPath() might return (wrong) absolute paths
			// for relative paths (extra / at beginning)
#ifdef WIN32
			fstream->set("filename", uri.getPath().substr(1));
#else
			fstream->set("filename", uri.getPath());
#endif
		}

		fstream->set("uri", uri.to_string());

		auto &recent_files = getConfig()["recent_files"];
		auto &file_details = recent_files[uri.getBaseURI()];
		std::string fname = uri.getPathSegment(-1);
		file_details["name"] = fname.substr(0, fname.find_last_of('.'));
		file_details["last_open"] = ftl::timer::get_time();

		_add_recent_source(uri);

		// TODO: URI normalization; should happen in add(,,) or add(,,,) take
		// ftl::URI instead of std::string as argument. Note the bug above.
		// TODO: write unit test for uri parsing
		add(fsid, uri.getBaseURI(), fstream);

		add_src_cb_.trigger(fsid);
		return fsid;
	}
	else if (scheme == ftl::URI::SCHEME_DEVICE) {
		int fsid = allocateFrameSetId("");  // TODO: Support groups with devices?
		fsid_lookup_[uri.getBaseURI()] = fsid; // Manually add mapping

		std::string srcname = std::string("source") + std::to_string(fsid);
		uri.to_json(getConfig()[srcname]);

		// Make the source object
		ftl::data::DiscreteSource *source;

		latest_[fsid] = nullptr;
		lk.unlock();

		if (uri.getBaseURI() == "device:render" || uri.getBaseURI() == "device:openvr") {
			auto *rsource = ftl::create<ftl::render::Source>(this, srcname, this);
			renderers_[fsid] = rsource;
			source = rsource;

			// Create local builder instance
			auto *creator = new ftl::streams::ManualSourceBuilder(pool_.get(), fsid, source);
			if (uri.getBaseURI() == "device:openvr") creator->setFrameRate(10000);
			else creator->setFrameRate(30);

			std::shared_ptr<ftl::streams::BaseBuilder> creatorptr(creator);
			lk.lock();
			receiver_->registerBuilder(creatorptr);

			// FIXME: pointer is deleted when removed from receiver
			render_builders_.push_back(creator);
		} else {
			auto *dsource = ftl::create<ftl::rgbd::Source>(this, srcname);
			devices_[fsid] = dsource;
			source = dsource;
			_createPipeline(fsid);

			// Create local builder instance
			auto *creator = new ftl::streams::IntervalSourceBuilder(pool_.get(), fsid, {source});
			std::shared_ptr<ftl::streams::BaseBuilder> creatorptr(creator);

			lk.lock();
			receiver_->registerBuilder(creatorptr);

			creator->start();
		}

		_add_recent_source(uri);

		add_src_cb_.trigger(fsid);
		return fsid;
	}

	else if ((scheme == ftl::URI::SCHEME_TCP) ||
			 (scheme == ftl::URI::SCHEME_WS)) {

		// just connect, onConnect callback will add the stream
		// TODO: do not connect same uri twice
		// TODO: write unit test

		auto &known_hosts = getConfig()["known_hosts"];
		auto &host_details = known_hosts[uri.getBaseURIWithUser()];
		host_details["last_open"] = ftl::timer::get_time();

		if (uri.getPathLength() == 1 && uri.getPathSegment(0) == "*") {
			auto *p = net_->connect(uri.getBaseURIWithUser());
			if (p->waitConnection()) {
				ftl::pool.push([this,p](int id) {_updateNetSources(p, true); });
			}
		} else {
			ftl::pool.push([this,path = uri.getBaseURIWithUser()](int id) { net_->connect(path)->noReconnect(); });
		}

	}
	else if (scheme == ftl::URI::SCHEME_FTL) {
		// Attempt to ensure connection first
		auto &known = getConfig()["recent_sources"];
		auto &details = known[uri.getBaseURI()];
		if (details.contains("host")) {
			auto *p = net_->connect(details["host"].get<std::string>());
			p->noReconnect();
			if (!p->waitConnection()) {
				throw FTL_Error("Could not connect to host " << details["host"].get<std::string>() << " for stream " << uri.getBaseURI());
			}
		} else {
			// See if it can otherwise be found?
			LOG(WARNING) << "Could not find stream host";
		}

		auto *stream = ftl::create<ftl::stream::Net>
			(this, std::string("netstream")
			+std::to_string(fsid_lookup_.size()), net_);

		int fsid = allocateFrameSetId(group);

		stream->set("uri", uri.to_string());
		add(fsid, uri.getBaseURI(), stream);

		LOG(INFO)	<< "Add Stream: "
					<< stream->value("uri", std::string("NONE"))
					<< " (" << fsid << ")";

		add_src_cb_.trigger(fsid);
		return fsid;
	} else if (scheme == ftl::URI::SCHEME_GROUP) {
		auto &known = getConfig()["known_groups"];
		if (known.contains(uri.getBaseURI())) {
			auto &sources = known[uri.getBaseURI()]["sources"];

			lk.unlock();
			for (auto i=sources.begin(); i!=sources.end(); ++i) {
				ftl::URI uri2(i.key());
				uri2.setAttribute("group", uri.getPathSegment(0));
				add(uri2);
			}

			lk.lock();
			_add_recent_source(uri);
		}
	}
	else{
		throw ftl::exception("bad uri");
	}
	return -1;
}

void Feed::render() {
	SHARED_LOCK(mtx_, lk);
	auto builders = render_builders_;
	lk.unlock();

	for (auto *r : builders) {
		r->tick();
	}
}

uint32_t Feed::getID(const std::string &source) {
	return fsid_lookup_.at(source);
}

const std::unordered_set<Channel> Feed::availableChannels(ftl::data::FrameID id) {
	ftl::data::FrameSetPtr fs;
	// FIXME: Should this be locked?
	std::atomic_store(&fs, latest_.at(id.frameset()));
	if (fs && fs->hasFrame(id.source())) {
		return (*fs.get())[id.source()].allChannels();
	}
	return {};
}

std::vector<ftl::data::FrameID> Feed::listFrames() {
	std::vector<ftl::data::FrameID> result;
	SHARED_LOCK(mtx_, lk);
	result.reserve(fsid_lookup_.size());
	for (const auto [k, fs] : latest_) {
		if (fs) {
			for (unsigned i = 0; i < fs->frames.size(); i++) {
				result.push_back(ftl::data::FrameID(k, i));
			}
		}
	}
	return result;
}

std::string Feed::getURI(uint32_t fsid) {
	SHARED_LOCK(mtx_, lk);
	for (const auto& [k, v] : fsid_lookup_) {
		if (v == fsid) {
			return k;
		}
	}
	return "";
}

std::string Feed::getSourceURI(ftl::data::FrameID id) {
	/*if (streams_.count(id.frameset())) {
		auto i = streams_.find(id.frameset());
		return i->second->getID();
	} else if (devices_.count(id.frameset())) {
		auto i = devices_.find(id.frameset());
		return i->second->getID();
	} else if (renderers_.count(id.frameset())) {
		auto i = renderers_.find(id.frameset());
		return i->second->getID();
	}*/

	return "";
}

std::vector<unsigned int> Feed::listFrameSets() {
	SHARED_LOCK(mtx_, lk);
	std::vector<unsigned int> result;
	result.reserve(fsid_lookup_.size());
	for (const auto [k, fs] : latest_) {
		if (fs) {
			result.push_back(k);
		}
	}
	return result;
}

void Feed::lowLatencyMode() {
	receiver_->set("frameset_buffer_size", 0);
}

// ==== Record =================================================================

void Feed::startRecording(Filter *f, const std::string &filename) {
	{
		UNIQUE_LOCK(mtx_, lk);
		if (_isRecording()) throw FTL_Error("Already recording, cannot record " << filename);

		record_filter_ = f;

		auto *fstream = ftl::create<ftl::stream::File>(this, "record_file");
		fstream->setMode(ftl::stream::File::Mode::Write);
		fstream->set("filename", filename);
		record_stream_->add(fstream);
		record_stream_->begin();
		recorder_->resetSender();
	}
	_beginRecord(f);
}

void Feed::startStreaming(Filter *f, const std::string &filename) {
	if (_isRecording()) throw FTL_Error("Already recording, cannot live stream: " << filename);

	// TODO: Allow net streaming
}

void Feed::startStreaming(Filter *f) {
	{
		UNIQUE_LOCK(mtx_, lk);
		if (_isRecording()) throw FTL_Error("Already recording, cannot live stream");

		record_filter_ = f;

		auto *nstream = ftl::create<ftl::stream::Net>(this, "live_stream", net_);
		nstream->set("uri", value("uri", std::string("ftl://vision.utu.fi/live")));

		record_new_client_ = nstream->onClientConnect([this](ftl::net::Peer *p) {
			stream_->reset();
			return true;
		});

		record_stream_->add(nstream);
		record_stream_->begin();
		recorder_->resetSender();
	}
	_beginRecord(f);
}

void Feed::_beginRecord(Filter *f) {

	handle_record_ = pool_->onFlushSet([this, f](ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
		// Skip framesets not in filter.
		if (!f->sources().empty() && f->sources().count(fs.frameset()) == 0) return true;

		if (f->channels().count(c)) {
			recorder_->post(fs, c);
		} else {
			recorder_->post(fs, c, true);
		}
		return true;
	});

	handle_record2_ = f->onWithHandle([this, f](const ftl::data::FrameSetPtr &fs) {
		record_stream_->select(fs->frameset(), f->channels(), true);
		stream_->select(fs->frameset(), f->channels(), true);
		ftl::pool.push([fs](int id) {
			try {
				fs->flush();  // Force now to reduce latency
			} catch (const ftl::exception &e) {
				LOG(ERROR) << "Exception when sending: " << e.what();
			}
		});
		return true;
	});
}

void Feed::stopRecording() {
	UNIQUE_LOCK(mtx_, lk);
	_stopRecording();
}

void Feed::_stopRecording() {
	handle_record_.cancel();
	handle_record2_.cancel();
	record_new_client_.cancel();
	record_stream_->end();

	auto garbage = record_stream_->streams();

	record_stream_->clear();

	for (auto *s : garbage) {
		delete s;
	}

	record_filter_ = nullptr;
}

bool Feed::isRecording() {
	SHARED_LOCK(mtx_, lk);
	return _isRecording();
}

bool Feed::_isRecording() {
	return record_stream_->streams().size() != 0;
}

ftl::data::FrameSetPtr Feed::getFrameSet(uint32_t fsid) {
	if (latest_.count(fsid) == 0) {
		throw ftl::exception("No FrameSet with given ID");
	}
	return std::atomic_load(&latest_[fsid]);
}
