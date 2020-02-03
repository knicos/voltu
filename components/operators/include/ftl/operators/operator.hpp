#ifndef _FTL_OPERATORS_OPERATOR_HPP_
#define _FTL_OPERATORS_OPERATOR_HPP_

#include <list>
#include <ftl/configurable.hpp>
#include <ftl/configuration.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * An abstract frame operator interface. Any kind of filter that operates on a
 * single frame should use this as a base class. An example of a filter would
 * be MLS smoothing, or optical flow temporal smoothing. Some 'filters' might
 * simply generate additional channels, such as a 'Normals' filter that
 * generates a normals channel. Filters may also have internal data buffers,
 * these may also persist between frames in some cases.
 */
class Operator {
	public:
	explicit Operator(ftl::Configurable *cfg);
	virtual ~Operator();

	enum class Type {
		OneToOne,		// Frame to Frame (Filter or generator)
		ManyToOne,		// FrameSet to Frame (Rendering or Compositing)
		ManyToMany,		// FrameSet to FrameSet (alignment)
		OneToMany		// Currently not used or supported
	};

	/**
	 * Must be implemented and return an operator structural type.
	 */
	virtual Type type() const =0;

	virtual bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream);
	virtual bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream);
	virtual bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, cudaStream_t stream);

	inline void enable() { enabled_ = true; }
	inline void disable() { enabled_ = false; }
	inline bool enabled() const { return enabled_; }
	inline void enabled(bool e) { enabled_ = e; }

	inline ftl::Configurable *config() const { return config_; }

	private:
	bool enabled_;
	ftl::Configurable *config_;
};

namespace detail {

struct ConstructionHelperBase {
	explicit ConstructionHelperBase(ftl::Configurable *cfg) : config(cfg) {}
	virtual ~ConstructionHelperBase() {}
	virtual ftl::operators::Operator *make()=0;

	ftl::Configurable *config;
};

template <typename T>
struct ConstructionHelper : public ConstructionHelperBase {
	explicit ConstructionHelper(ftl::Configurable *cfg) : ConstructionHelperBase(cfg) {}
	~ConstructionHelper() {}
	ftl::operators::Operator *make() override {
		return new T(config);
	}
};

template <typename T, typename... ARGS>
struct ConstructionHelper2 : public ConstructionHelperBase {
	explicit ConstructionHelper2(ftl::Configurable *cfg, ARGS... args) : ConstructionHelperBase(cfg) {
		arguments_ = std::make_tuple(args...);
	}
	~ConstructionHelper2() {}
	ftl::operators::Operator *make() override {
		return new T(config, arguments_);
	}

	private:
	std::tuple<ARGS...> arguments_;
};

struct OperatorNode {
	ConstructionHelperBase *maker;
	std::vector<ftl::operators::Operator*> instances;
};

}

/**
 * Represent a sequential collection of operators. Each operator created is
 * added to an internal list and then applied to a frame in the order they were
 * created. A directed acyclic graph can also be formed.
 */
class Graph : public ftl::Configurable {
	public:
	explicit Graph(nlohmann::json &config);
	~Graph();

	template <typename T>
	ftl::Configurable *append(const std::string &name);

	template <typename T, typename... ARGS>
	ftl::Configurable *append(const std::string &name, ARGS...);

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream=0);
	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream=0);
	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, cudaStream_t stream=0);

	cudaStream_t getStream() const { return stream_; }

	private:
	std::list<ftl::operators::detail::OperatorNode> operators_;
	std::map<std::string, ftl::Configurable*> configs_;
	cudaStream_t stream_;

	ftl::Configurable *_append(ftl::operators::detail::ConstructionHelperBase*);
};

}
}

template <typename T>
ftl::Configurable *ftl::operators::Graph::append(const std::string &name) {
	if (configs_.find(name) == configs_.end()) {
		configs_[name] = ftl::create<ftl::Configurable>(this, name);
	}
	return _append(new ftl::operators::detail::ConstructionHelper<T>(configs_[name]));
}

template <typename T, typename... ARGS>
ftl::Configurable *ftl::operators::Graph::append(const std::string &name, ARGS... args) {
	if (configs_.find(name) == configs_.end()) {
		configs_[name] = ftl::create<ftl::Configurable>(this, name);
	}
	return _append(new ftl::operators::detail::ConstructionHelper2<T, ARGS...>(configs_[name], args...));
}

#endif  // _FTL_OPERATORS_OPERATOR_HPP_
