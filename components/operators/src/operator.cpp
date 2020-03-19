#include <ftl/operators/operator.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::Operator;
using ftl::operators::Graph;
using ftl::rgbd::Frame;
using ftl::rgbd::FrameSet;
using ftl::rgbd::Source;

Operator::Operator(ftl::Configurable *config) : config_(config) {
	enabled_ = config_->value("enabled", true);

	config_->on("enabled", [this](const ftl::config::Event &e) {
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
}

Graph::~Graph() {
	cudaStreamDestroy(stream_);
}

bool Graph::apply(FrameSet &in, FrameSet &out, cudaStream_t stream) {
	if (!value("enabled", true)) return false;

	auto stream_actual = (stream == 0) ? stream_ : stream;
	bool success = true;

	if (in.frames.size() != out.frames.size()) return false;

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
				if (!in.hasFrame(j)) continue;
				
				int iix = (i.instances[0]->isMemoryHeavy()) ? 0 : j&0x1;
				auto *instance = i.instances[iix];

				if (instance->enabled()) {
					try {
						instance->apply(in.frames[j], out.frames[j], stream_actual);
					} catch (const std::exception &e) {
						LOG(ERROR) << "Operator exception for '" << instance->config()->getID() << "': " << e.what();
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
				break;
			}
		}
	}

	success = waitAll(stream_actual) && success;

	if (stream == 0) {
		cudaSafeCall(cudaStreamSynchronize(stream_actual));
	}

	return success;
}

ftl::Configurable *Graph::_append(ftl::operators::detail::ConstructionHelperBase *m) {
	auto &o = operators_.emplace_back();
	o.maker = m;
	return m->config;
}
