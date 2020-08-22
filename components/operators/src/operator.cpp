#include <ftl/operators/operator.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::Operator;
using ftl::operators::Graph;
using ftl::rgbd::Frame;
using ftl::rgbd::FrameSet;
using ftl::rgbd::Source;
using ftl::codecs::Channel;

Operator::Operator(ftl::operators::Graph *g, ftl::Configurable *config) : config_(config), graph_(g) {
	enabled_ = config_->value("enabled", true);

	config_->on("enabled", [this]() {
		enabled_ = config_->value("enabled", true);
	});
}

Operator::~Operator() {}

bool Operator::apply(Frame &in, Frame &out, cudaStream_t stream) {
	throw FTL_Error("Operation application to frame not supported");
}

bool Operator::apply(FrameSet &in, FrameSet &out, cudaStream_t stream) {
	throw FTL_Error("Operation application to frameset not supported");
}

bool Operator::apply(FrameSet &in, Frame &out, cudaStream_t stream) {
	throw FTL_Error("Operation application as a reduction not supported");
}



Graph::Graph(nlohmann::json &config) : ftl::Configurable(config) {
	busy_.clear();
}

Graph::~Graph() {
	// Cleanup configurables
	for (auto &c : configs_) {
		delete c.second;
	}
	for (auto &o : operators_) {
		for (auto *i : o.instances) {
			delete i;
		}
	}
}

cv::cuda::GpuMat &Graph::createBuffer(ftl::operators::Buffer b, uint32_t fid) {
	if (fid > 32) throw FTL_Error("Too many frames for buffer");
	auto &v = buffers_[(uint32_t(b) << 8) + fid];
	valid_buffers_.insert((uint32_t(b) << 8) + fid);
	return v;
}

cv::cuda::GpuMat &Graph::getBuffer(ftl::operators::Buffer b, uint32_t fid) {
	if (fid > 32) throw FTL_Error("Too many frames for buffer");
	if (!hasBuffer(b, fid)) throw FTL_Error("Buffer does not exist: " << int(b));
	auto &v = buffers_.at((uint32_t(b) << 8) + fid);
	return v;
}

bool Graph::hasBuffer(ftl::operators::Buffer b, uint32_t fid) const {
	return valid_buffers_.count((uint32_t(b) << 8) + fid) > 0;
}

bool Graph::queue(const ftl::data::FrameSetPtr &fs, const std::function<void()> &cb) {
	if (!value("enabled", true)) return true;
	if (fs->frames.size() < 1) return true;

	{
		UNIQUE_LOCK(mtx_, lk);
		if (queue_.size() > 3) {
			LOG(ERROR) << "Pipeline queue exceeded";
			return false;
		}
		queue_.emplace_back(fs, cb);
	}

	if (busy_.test_and_set()) {
		LOG(INFO) << "Pipeline queued... " << queue_.size();
		return true;
	}

	_processOne();
	return true;
}

bool Graph::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out) {
	if (!value("enabled", true)) return true;
	if (in.frames.size() < 1) return true;

	return _apply(in, out);
}

bool Graph::_apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out) {
	auto stream_actual = in.frames[0].stream();
	bool success = true;

	valid_buffers_.clear();

	for (auto &f : out.frames) {
		if (!f.hasOwn(Channel::Pipelines)) f.create<std::list<std::string>>(Channel::Pipelines);
		auto pls = f.set<std::list<std::string>>(Channel::Pipelines);
		pls = getID();
	}

	for (auto &i : operators_) {
		if (i.instances.size() < 1) {
			i.instances.push_back(i.maker->make(this));
		}

		if (i.instances[0]->type() == Operator::Type::OneToOne) {
			// Make sure there are enough instances
			//while (i.instances.size() < in.frames.size()) {
				//i.instances.push_back(i.maker->make());
			//}
			if (in.frames.size() > 1 && i.instances.size() < 2 && !i.instances[0]->isMemoryHeavy()) {
				i.instances.push_back(i.maker->make(this));
			}

			for (size_t j=0; j<in.frames.size(); ++j) {
				if (!in.hasFrame(j)) in.frames[j].message(ftl::data::Message::Warning_INCOMPLETE_FRAME, "Frame not complete in Pipeline");
				
				int iix = (i.instances[0]->isMemoryHeavy()) ? 0 : j&0x1;
				auto *instance = i.instances[iix];

				if (instance->enabled()) {
					try {
						instance->apply(in.frames[j].cast<ftl::rgbd::Frame>(), out.frames[j].cast<ftl::rgbd::Frame>(), stream_actual);
						//cudaSafeCall(cudaStreamSynchronize(stream_actual));
					} catch (const std::exception &e) {
						LOG(ERROR) << "Operator exception for '" << instance->config()->getID() << "': " << e.what();
						in.frames[j].message(ftl::data::Message::Error_OPERATOR_EXCEPTION, "Operator exception");
						success = false;
						break;
					}
				}
			}
			if (!success) break;
		} else if (i.instances[0]->type() == Operator::Type::ManyToMany) {
			auto *instance = i.instances[0];

			if (instance->enabled()) {
				try {
					instance->apply(in, out, stream_actual);
					//cudaSafeCall(cudaStreamSynchronize(stream_actual));
				} catch (const std::exception &e) {
					LOG(ERROR) << "Operator exception for '" << instance->config()->getID() << "': " << e.what();
					if (in.frames.size() > 0) in.frames[0].message(ftl::data::Message::Error_OPERATOR_EXCEPTION, "Operator exception");
					success = false;
					break;
				}
			}
		}
	}

	success = waitAll(stream_actual) && success;
	return success;
}

void Graph::_processOne() {

	ftl::data::FrameSetPtr fs;
	std::function<void()> cb;

	{
		UNIQUE_LOCK(mtx_, lk);
		if(queue_.size() == 0) {
			busy_.clear();
			return;
		}

		fs = queue_.front().first;
		cb = queue_.front().second;
		queue_.pop_front();
	}

	auto &in = *fs;
	auto &out = *fs;

	auto stream_actual = in.frames[0].stream();

	_apply(in, out);

	if (cb) {
		cudaCallback(stream_actual, [this,cb]() {
			bool sched = false;
			{
				UNIQUE_LOCK(mtx_, lk);
				if (queue_.size() == 0) busy_.clear();
				else sched = true;
			}
			ftl::pool.push([this,cb,sched](int id) {
				if (sched) _processOne();
				cb();
			});
		});
	} else {
		busy_.clear();
	}
	
	return;
}

bool Graph::waitAll(cudaStream_t stream) {
	for (auto &i : operators_) {
		for (auto *j : i.instances) {
			try {
				j->wait(stream);
			} catch (std::exception &e) {
				LOG(ERROR) << "Operator exception for '" << j->config()->getID() << "': " << e.what();
				return false;
			}
		}
	}
	return true;
}

bool Graph::apply(Frame &in, Frame &out, const std::function<void()> &cb) {
	if (!value("enabled", true)) return true;

	auto stream_actual = in.stream();
	bool success = true;

	if (busy_.test_and_set()) {
		LOG(ERROR) << "Pipeline already in use: " << in.timestamp();
		//if (cb) cb();
		return false;
	}

	valid_buffers_.clear();

	if (!out.hasOwn(Channel::Pipelines)) out.create<std::list<std::string>>(Channel::Pipelines);
	auto pls = out.set<std::list<std::string>>(Channel::Pipelines);
	pls = getID();

	for (auto &i : operators_) {
		// Make sure there are enough instances
		if (i.instances.size() < 1) {
			i.instances.push_back(i.maker->make(this));
		}

		auto *instance = i.instances[0];

		if (instance->enabled()) {
			try {
				instance->apply(in, out, stream_actual);
				//cudaSafeCall(cudaStreamSynchronize(stream_actual));
			} catch (const std::exception &e) {
				LOG(ERROR) << "Operator exception for '" << instance->config()->getID() << "': " << e.what();
				success = false;
				out.message(ftl::data::Message::Error_OPERATOR_EXCEPTION, "Operator exception");
				break;
			}
		}
	}

	success = waitAll(stream_actual) && success;

	if (cb) {
		cudaCallback(stream_actual, [this,cb]() {
			busy_.clear();
			ftl::pool.push([cb](int id) { cb(); });
		});
	} else {
		//cudaSafeCall(cudaStreamSynchronize(stream_actual));
		busy_.clear();
	}

	//busy_.clear();
	return true;
}

ftl::Configurable *Graph::_append(ftl::operators::detail::ConstructionHelperBase *m) {
	auto &o = operators_.emplace_back();
	o.maker = m;
	return m->config;
}
