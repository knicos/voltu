#include <ftl/operators/operator.hpp>

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

bool Operator::apply(Frame &in, Frame &out, Source *s, cudaStream_t stream) {
	return false;
}

bool Operator::apply(FrameSet &in, FrameSet &out, cudaStream_t stream) {
	return false;
}

bool Operator::apply(FrameSet &in, Frame &out, Source *os, cudaStream_t stream) {
	return false;
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

	if (in.frames.size() != out.frames.size()) return false;

	for (auto &i : operators_) {
		// Make sure there are enough instances
		while (i.instances.size() < in.frames.size()) {
			i.instances.push_back(i.maker->make());
		}

		for (int j=0; j<in.frames.size(); ++j) {
			auto *instance = i.instances[j];

			if (instance->enabled()) {
				instance->apply(in.frames[j], out.frames[j], in.sources[j], stream_actual);
			}
		}
	}

	if (stream == 0) {
		cudaStreamSynchronize(stream_actual);
		cudaSafeCall( cudaGetLastError() );
	}

	return true;
}

bool Graph::apply(Frame &in, Frame &out, Source *s, cudaStream_t stream) {
	if (!value("enabled", true)) return false;

	for (auto &i : operators_) {
		// Make sure there are enough instances
		if (i.instances.size() < 1) {
			i.instances.push_back(i.maker->make());
		}

		auto *instance = i.instances[0];

		if (instance->enabled()) {
			instance->apply(in, out, s, stream);
		}
	}

	return true;
}

ftl::Configurable *Graph::_append(ftl::operators::detail::ConstructionHelperBase *m) {
	auto &o = operators_.emplace_back();
	o.maker = m;
	return m->config;
}
