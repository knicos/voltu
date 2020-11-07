#include <ftl/operators/fusion.hpp>
#include <ftl/operators/cuda/carver.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

using ftl::operators::Fusion;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;

Fusion::Fusion(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg), mls_(3) {

}

Fusion::~Fusion() {

}

bool Fusion::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
	float mls_smoothing = config()->value("mls_smoothing", 0.01f);
	int mls_iters = config()->value("mls_iterations", 2);

	if (weights_.size() != in.frames.size()) weights_.resize(in.frames.size());

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	for (size_t i=0; i<in.frames.size(); ++i) {
		if (!in.hasFrame(i)) continue;
		const GpuMat &col = in.frames[i].get<GpuMat>(Channel::Colour);
		const GpuMat &d = in.frames[i].get<GpuMat>(Channel::Depth);

		cv::cuda::cvtColor(col, temp_, cv::COLOR_BGRA2GRAY, 0, cvstream);
		cv::cuda::resize(temp_, weights_[i], d.size(), 0, 0, cv::INTER_LINEAR, cvstream);
	}

	if (config()->value("visibility_carving", true)) {
		for (size_t i=0; i < in.frames.size(); ++i) {
			if (!in.hasFrame(i)) continue;

			auto &f = in.frames[i].cast<ftl::rgbd::Frame>();

			for (size_t j=0; j < in.frames.size(); ++j) {
				if (i == j) continue;
				if (!in.hasFrame(j)) continue;

				auto &ref = in.frames[j].cast<ftl::rgbd::Frame>();

				auto transformR = MatrixConversion::toCUDA(ref.getPose().cast<float>().inverse() * f.getPose().cast<float>());

				ftl::cuda::depth_carve(
					f.create<cv::cuda::GpuMat>(Channel::Depth),
					ref.get<cv::cuda::GpuMat>(Channel::Depth),
					//f.get<cv::cuda::GpuMat>(Channel::Colour),
					//ref.get<cv::cuda::GpuMat>(Channel::Colour),
					//colour_scale_,
					transformR,
					f.getLeft(),
					ref.getLeft(),
					stream
				);
			}

			//ftl::cuda::apply_colour_scaling(colour_scale_, f.create<cv::cuda::GpuMat>(Channel::Colour), 3, stream_);
		}
	}

	for (int iters=0; iters < mls_iters; ++iters) {
	for (size_t i=0; i<in.frames.size(); ++i) {
		if (!in.hasFrame(i)) continue;

		auto &f1 = in.frames[i].cast<ftl::rgbd::Frame>();

		Eigen::Vector4d d1(0.0, 0.0, 1.0, 0.0);
		d1 = f1.getPose() * d1;

		auto pose1 = MatrixConversion::toCUDA(f1.getPose().cast<float>());

		mls_.prime(
			f1.get<GpuMat>(Channel::Depth),
			weights_[i],
			f1.getLeft(),
			pose1,
			stream
		);

		for (size_t j=0; j<in.frames.size(); ++j) {
			if (!in.hasFrame(j)) continue;
			//if (i == j) continue;

			//LOG(INFO) << "Running phase1";

			auto &f2 = in.frames[j].cast<ftl::rgbd::Frame>();

			// Are cameras facing similar enough direction?
			Eigen::Vector4d d2(0.0, 0.0, 1.0, 0.0);
			d2 = f2.getPose() * d2;
			// No, so skip this combination
			if (d1.dot(d2) <= 0.0) continue;

			auto pose2 = MatrixConversion::toCUDA(f2.getPose().cast<float>());

			mls_.gather(
				f2.get<GpuMat>(Channel::Depth),
				f2.get<GpuMat>(Channel::Normals),
				weights_[j],
				f2.getLeft(),
				pose2,
				mls_smoothing,
				stream
			);
		}

		mls_.adjust(
			f1.create<GpuMat>(Channel::Depth),
			f1.create<GpuMat>(Channel::Normals),
			stream
		);
	}
	}

	return true;
}