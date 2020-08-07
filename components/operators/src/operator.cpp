#include <ftl/operators/operator.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::Operator;
using ftl::operators::Graph;
using ftl::rgbd::Frame;
using ftl::rgbd::FrameSet;
using ftl::rgbd::Source;
using ftl::codecs::Channel;

Operator::Operator(ftl::Configurable *config) : config_(config) {
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
	cudaSafeCall( cudaStreamCreate(&stream_) );
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
	cudaStreamDestroy(stream_);
}

bool Graph::apply(FrameSet &in, FrameSet &out, cudaStream_t stream) {
	if (!value("enabled", true)) return false;

	auto stream_actual = (stream == 0) ? stream_ : stream;
	bool success = true;

	if (in.frames.size() != out.frames.size()) return false;

	if (busy_.test_and_set()) {
		LOG(ERROR) << "Pipeline already in use: " << in.timestamp();
		return false;
	}

	for (auto &f : out.frames) {
		if (!f.hasOwn(Channel::Pipelines)) f.create<std::list<std::string>>(Channel::Pipelines);
		auto pls = f.set<std::list<std::string>>(Channel::Pipelines);
		pls = getID();
	}

	for (auto &i : operators_) {
		if (i.instances.size() < 1) {
			i.instances.push_back(i.maker->make());
		}

		if (i.instances[0]->type() == Operator::Type::OneToOne) {
			// Make sure there are enough instances
			//while (i.instances.size() < in.frames.size()) {
				//i.instances.push_back(i.maker->make());
			//}
			if (in.frames.size() > 1 && i.instances.size() < 2 && !i.instances[0]->isMemoryHeavy()) {
				i.instances.push_back(i.maker->make());
			}

			for (size_t j=0; j<in.frames.size(); ++j) {
				if (!in.hasFrame(j)) in.frames[j].message(ftl::data::Message::Warning_INCOMPLETE_FRAME, "Frame not complete in Pipeline");
				
				int iix = (i.instances[0]->isMemoryHeavy()) ? 0 : j&0x1;
				auto *instance = i.instances[iix];

				if (instance->enabled()) {
					try {
						instance->apply(in.frames[j].cast<ftl::rgbd::Frame>(), out.frames[j].cast<ftl::rgbd::Frame>(), stream_actual);
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

	if (stream == 0) {
		cudaSafeCall(cudaStreamSynchronize(stream_actual));
	}

	busy_.clear();
	return success;
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

bool Graph::apply(Frame &in, Frame &out, cudaStream_t stream) {
	if (!value("enabled", true)) return false;

	auto stream_actual = (stream == 0) ? stream_ : stream;
	bool success = true;

	if (busy_.test_and_set()) {
		LOG(ERROR) << "Pipeline already in use: " << in.timestamp();
		return false;
	}

	if (!out.hasOwn(Channel::Pipelines)) out.create<std::list<std::string>>(Channel::Pipelines);
	auto pls = out.set<std::list<std::string>>(Channel::Pipelines);
	pls = getID();

	for (auto &i : operators_) {
		// Make sure there are enough instances
		if (i.instances.size() < 1) {
			i.instances.push_back(i.maker->make());
		}

		auto *instance = i.instances[0];

		if (instance->enabled()) {
			try {
				instance->apply(in, out, stream_actual);
			} catch (const std::exception &e) {
				LOG(ERROR) << "Operator exception for '" << instance->config()->getID() << "': " << e.what();
				success = false;
				out.message(ftl::data::Message::Error_OPERATOR_EXCEPTION, "Operator exception");
				break;
			}
		}
	}

	success = waitAll(stream_actual) && success;

	if (stream == 0) {
		cudaSafeCall(cudaStreamSynchronize(stream_actual));
	}

	busy_.clear();
	return success;
}

ftl::Configurable *Graph::_append(ftl::operators::detail::ConstructionHelperBase *m) {
	auto &o = operators_.emplace_back();
	o.maker = m;
	return m->config;
}
