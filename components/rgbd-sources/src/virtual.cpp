#include <ftl/rgbd/virtual.hpp>

using ftl::rgbd::VirtualSource;
using ftl::rgbd::Source;
using ftl::rgbd::Channel;

class VirtualImpl : public ftl::rgbd::detail::Source {
	public:
	explicit VirtualImpl(ftl::rgbd::Source *host, const ftl::rgbd::Camera &params) : ftl::rgbd::detail::Source(host) {
		params_ = params;
		capabilities_ = ftl::rgbd::kCapVideo | ftl::rgbd::kCapStereo;
		if (!host->value("locked", false)) capabilities_ |= ftl::rgbd::kCapMovable;
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

	bool isReady() override { return true; }

	std::function<void(ftl::rgbd::Frame &)> callback;
	ftl::rgbd::Frame frame;
};

VirtualSource::VirtualSource(ftl::config::json_t &cfg) : Source(cfg) {
	auto params = params_;
	impl_ = new VirtualImpl(this, params);
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