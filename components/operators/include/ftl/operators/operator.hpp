#ifndef _FTL_OPERATORS_OPERATOR_HPP_
#define _FTL_OPERATORS_OPERATOR_HPP_

#include <list>
#include <ftl/configurable.hpp>
#include <ftl/configuration.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/operators/buffer.hpp>

namespace ftl {
namespace operators {

class Graph;

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
	Operator(Graph *pgraph, ftl::Configurable *cfg);
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

	/**
	 * In case this operator operates async, force a wait for complete when
	 * this is called. Useful for CPU processing in parallel to GPU. A CUDA
	 * stream is passed that corresponds to the pipeline that called wait,
	 * allowing any GPU parallel streams to insert a wait event.
	 */
	virtual void wait(cudaStream_t) {}

	virtual bool isMemoryHeavy() const { return false; }

	inline ftl::Configurable *config() const { return config_; }

	inline Graph *graph() const { return graph_; }

	private:
	bool enabled_;
	ftl::Configurable *config_;
	Graph *graph_;
};

namespace detail {

struct ConstructionHelperBase {
	explicit ConstructionHelperBase(ftl::Configurable *cfg) : config(cfg) {}
	virtual ~ConstructionHelperBase() {}
	virtual ftl::operators::Operator *make(Graph *g)=0;

	ftl::Configurable *config;
};

template <typename T>
struct ConstructionHelper : public ConstructionHelperBase {
	explicit ConstructionHelper(ftl::Configurable *cfg) : ConstructionHelperBase(cfg) { T::configuration(cfg); }
	~ConstructionHelper() {}
	ftl::operators::Operator *make(Graph *g) override {
		return new T(g, config);
	}
};

template <typename T, typename... ARGS>
struct ConstructionHelper2 : public ConstructionHelperBase {
	explicit ConstructionHelper2(ftl::Configurable *cfg, ARGS... args) : ConstructionHelperBase(cfg) {
		arguments_ = std::make_tuple(args...);
		T::configuration(cfg);
	}
	~ConstructionHelper2() {}
	ftl::operators::Operator *make(Graph *g) override {
		return new T(g, config, arguments_);
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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, const std::function<void()> &cb=nullptr);

	bool queue(const ftl::data::FrameSetPtr &fs, const std::function<void()> &cb);

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out);
	//bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, const std::function<void()> &cb=nullptr);

	/**
	 * Make sure all async operators have also completed. This is automatically
	 * called by `apply` so should not be needed unless an `applyAsync` is
	 * added in the future. The stream passed is the primary pipeline stream,
	 * which may be either `getStream()` or the stream argument passed to the
	 * `apply` method called.
	 */
	bool waitAll(cudaStream_t);

	inline cv::cuda::GpuMat &createBuffer(ftl::operators::Buffer b) { return createBuffer(b, 0); }
	cv::cuda::GpuMat &createBuffer(ftl::operators::Buffer b, uint32_t fid);

	cv::cuda::GpuMat &getBuffer(ftl::operators::Buffer b, uint32_t fid);

	bool hasBuffer(ftl::operators::Buffer b, uint32_t fid) const;

	private:
	std::list<ftl::operators::detail::OperatorNode> operators_;
	std::map<std::string, ftl::Configurable*> configs_;
	std::atomic_flag busy_;
	std::unordered_map<uint32_t,cv::cuda::GpuMat> buffers_;
	std::unordered_set<uint32_t> valid_buffers_;
	std::function<void()> callback_;
	std::list<std::pair<ftl::data::FrameSetPtr, std::function<void()>>> queue_;
	MUTEX mtx_;

	ftl::Configurable *_append(ftl::operators::detail::ConstructionHelperBase*);
	void _processOne();
	bool _apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out);
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
