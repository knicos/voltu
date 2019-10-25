#include <ftl/rgbd/virtual.hpp>

using ftl::rgbd::VirtualSource;
using ftl::rgbd::Source;
using ftl::codecs::Channel;
using ftl::rgbd::Camera;

class VirtualImpl : public ftl::rgbd::detail::Source {
	public:
	explicit VirtualImpl(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
		params_.width = host->value("width", 1280);
		params_.height = host->value("height", 720);
		params_.fx = host->value("focal", 700.0f);
		params_.fy = params_.fx;
		params_.cx = -(double)params_.width / 2.0;
		params_.cy = -(double)params_.height / 2.0;
		params_.minDepth = host->value("minDepth", 0.1f);
		params_.maxDepth = host->value("maxDepth", 20.0f);
		params_.doffs = 0;
		params_.baseline = host->value("baseline", 0.0f);

		params_right_.width = host->value("width", 1280);
		params_right_.height = host->value("height", 720);
		params_right_.fx = host->value("focal_right", 700.0f);
		params_right_.fy = params_right_.fx;
		params_right_.cx = host->value("centre_x_right", -(double)params_.width / 2.0);
		params_right_.cy = host->value("centre_y_right", -(double)params_.height / 2.0);
		params_right_.minDepth = host->value("minDepth", 0.1f);
		params_right_.maxDepth = host->value("maxDepth", 20.0f);
		params_right_.doffs = 0;
		params_right_.baseline = host->value("baseline", 0.0f);

		capabilities_ = ftl::rgbd::kCapVideo | ftl::rgbd::kCapStereo;

		if (!host->value("locked", false)) capabilities_ |= ftl::rgbd::kCapMovable;
		host->on("baseline", [this](const ftl::config::Event&) {
			params_.baseline = host_->value("baseline", 0.0f);
		});
		
		host->on("focal", [this](const ftl::config::Event&) {
			params_.fx = host_->value("focal", 700.0f);
			params_.fy = params_.fx;
		});

		host->on("centre_x", [this](const ftl::config::Event&) {
			params_.cx = host_->value("centre_x", 0.0f);
		});

		host->on("centre_y", [this](const ftl::config::Event&) {
			params_.cy = host_->value("centre_y", 0.0f);
		});

		host->on("focal_right", [this](const ftl::config::Event&) {
			params_right_.fx = host_->value("focal_right", 700.0f);
			params_right_.fy = params_right_.fx;
		});

		host->on("centre_x_right", [this](const ftl::config::Event&) {
			params_right_.cx = host_->value("centre_x_right", 0.0f);
		});

		host->on("centre_y_right", [this](const ftl::config::Event&) {
			params_right_.cy = host_->value("centre_y_right", 0.0f);
		});

	}

	~VirtualImpl() {

	}

	bool capture(int64_t ts) override {
		timestamp_ = ts;
		return true;
	}

	bool retrieve() override {
		return true;
	}

	bool compute(int n, int b) override {
		if (callback) {
			frame.reset();

			try {
				callback(frame);
			} catch (std::exception &e) {
				LOG(ERROR) << "Exception in render callback: " << e.what();
			} catch (...) {
				LOG(ERROR) << "Unknown exception in render callback";
			}

			if (frame.hasChannel(Channel::Colour)) {
				frame.download(Channel::Colour);
				cv::swap(frame.get<cv::Mat>(Channel::Colour), rgb_);	
			} else {
				LOG(ERROR) << "Channel 1 frame in rendering";
			}
			
			if ((host_->getChannel() != Channel::None) &&
					frame.hasChannel(host_->getChannel())) {
				frame.download(host_->getChannel());
				cv::swap(frame.get<cv::Mat>(host_->getChannel()), depth_);
			}

			auto cb = host_->callback();
			if (cb) cb(timestamp_, rgb_, depth_);
		}
		return true;
	}

	Camera parameters(ftl::codecs::Channel c) {
		return (c == Channel::Left) ? params_ : params_right_;
	}

	bool isReady() override { return true; }

	std::function<void(ftl::rgbd::Frame &)> callback;
	ftl::rgbd::Frame frame;

	ftl::rgbd::Camera params_right_;
};

VirtualSource::VirtualSource(ftl::config::json_t &cfg) : Source(cfg) {
	impl_ = new VirtualImpl(this);
}

VirtualSource::~VirtualSource() {

}

void VirtualSource::onRender(const std::function<void(ftl::rgbd::Frame &)> &f) {
	dynamic_cast<VirtualImpl*>(impl_)->callback = f;
}

/*
void Source::writeFrames(int64_t ts, const cv::Mat &rgb, const cv::Mat &depth) {
	if (!impl_) {
		UNIQUE_LOCK(mutex_,lk);
		rgb.copyTo(rgb_);
		depth.copyTo(depth_);
		timestamp_ = ts;
	}
}

void Source::writeFrames(int64_t ts, const ftl::cuda::TextureObject<uchar4> &rgb, const ftl::cuda::TextureObject<uint> &depth, cudaStream_t stream) {
	if (!impl_) {
		UNIQUE_LOCK(mutex_,lk);
		timestamp_ = ts;
		rgb_.create(rgb.height(), rgb.width(), CV_8UC4);
		cudaSafeCall(cudaMemcpy2DAsync(rgb_.data, rgb_.step, rgb.devicePtr(), rgb.pitch(), rgb_.cols * sizeof(uchar4), rgb_.rows, cudaMemcpyDeviceToHost, stream));
		depth_.create(depth.height(), depth.width(), CV_32SC1);
		cudaSafeCall(cudaMemcpy2DAsync(depth_.data, depth_.step, depth.devicePtr(), depth.pitch(), depth_.cols * sizeof(uint), depth_.rows, cudaMemcpyDeviceToHost, stream));
		//cudaSafeCall(cudaStreamSynchronize(stream));  // TODO:(Nick) Don't wait here.
		stream_ = stream;
		//depth_.convertTo(depth_, CV_32F, 1.0f / 1000.0f);
	} else {
		LOG(ERROR) << "writeFrames cannot be done on this source: " << getURI();
	}
}

void Source::writeFrames(int64_t ts, const ftl::cuda::TextureObject<uchar4> &rgb, const ftl::cuda::TextureObject<float> &depth, cudaStream_t stream) {
	if (!impl_) {
		UNIQUE_LOCK(mutex_,lk);
		timestamp_ = ts;
		rgb.download(rgb_, stream);
		//rgb_.create(rgb.height(), rgb.width(), CV_8UC4);
		//cudaSafeCall(cudaMemcpy2DAsync(rgb_.data, rgb_.step, rgb.devicePtr(), rgb.pitch(), rgb_.cols * sizeof(uchar4), rgb_.rows, cudaMemcpyDeviceToHost, stream));
		depth.download(depth_, stream);
		//depth_.create(depth.height(), depth.width(), CV_32FC1);
		//cudaSafeCall(cudaMemcpy2DAsync(depth_.data, depth_.step, depth.devicePtr(), depth.pitch(), depth_.cols * sizeof(float), depth_.rows, cudaMemcpyDeviceToHost, stream));
		
		stream_ = stream;
		cudaSafeCall(cudaStreamSynchronize(stream_));
		cv::cvtColor(rgb_,rgb_, cv::COLOR_BGRA2BGR);
		cv::cvtColor(rgb_,rgb_, cv::COLOR_Lab2BGR);

		if (callback_) callback_(timestamp_, rgb_, depth_);
	}
}

void Source::writeFrames(int64_t ts, const ftl::cuda::TextureObject<uchar4> &rgb, const ftl::cuda::TextureObject<uchar4> &rgb2, cudaStream_t stream) {
	if (!impl_) {
		UNIQUE_LOCK(mutex_,lk);
		timestamp_ = ts;
		rgb.download(rgb_, stream);
		//rgb_.create(rgb.height(), rgb.width(), CV_8UC4);
		//cudaSafeCall(cudaMemcpy2DAsync(rgb_.data, rgb_.step, rgb.devicePtr(), rgb.pitch(), rgb_.cols * sizeof(uchar4), rgb_.rows, cudaMemcpyDeviceToHost, stream));
		rgb2.download(depth_, stream);
		//depth_.create(depth.height(), depth.width(), CV_32FC1);
		//cudaSafeCall(cudaMemcpy2DAsync(depth_.data, depth_.step, depth.devicePtr(), depth.pitch(), depth_.cols * sizeof(float), depth_.rows, cudaMemcpyDeviceToHost, stream));
		
		stream_ = stream;
		cudaSafeCall(cudaStreamSynchronize(stream_));
		cv::cvtColor(rgb_,rgb_, cv::COLOR_BGRA2BGR);
		cv::cvtColor(rgb_,rgb_, cv::COLOR_Lab2BGR);
		cv::cvtColor(depth_,depth_, cv::COLOR_BGRA2BGR);
		cv::cvtColor(depth_,depth_, cv::COLOR_Lab2BGR);
	}
}
*/