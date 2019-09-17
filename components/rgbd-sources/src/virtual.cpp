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