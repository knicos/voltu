#include <ftl/operators/smoothing.hpp>
#include "smoothing_cuda.hpp"

#include <ftl/cuda/normals.hpp>

using ftl::operators::HFSmoother;
using ftl::operators::SimpleMLS;
using ftl::operators::ColourMLS;
using ftl::operators::AdaptiveMLS;
using ftl::operators::SmoothChannel;
using ftl::codecs::Channel;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

HFSmoother::HFSmoother(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

HFSmoother::~HFSmoother() {

}

bool HFSmoother::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
    float var_thresh = config()->value("variance_threshold", 0.0002f);
    int levels = max(0, min(config()->value("levels",0), 4));
    int iters = config()->value("iterations",5);

	// FIXME: in and out are assumed to be the same

    for (int i=0; i<iters; ++i) {
        ftl::cuda::smoothing_factor(
            in.createTexture<float>(Channel::Depth),
            in.createTexture<float>(Channel::Energy, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            in.createTexture<float>(Channel::Smoothing, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            var_thresh,
            s->parameters(), 0
        );
    }

    //LOG(INFO) << "PARAMS DEPTHS  " << s->parameters().minDepth << "," << s->parameters().maxDepth;

    /*for (int i=0; i<levels; ++i) {
        var_thresh *= 2.0f;
        auto &dmat = f.get<GpuMat>(Channel::Depth); 
        cv::cuda::resize(dmat, frames_[i].create<GpuMat>(Channel::Depth), cv::Size(dmat.cols / (2*(i+1)), dmat.rows / (2*(i+1))), 0.0, 0.0, cv::INTER_NEAREST);

        ftl::cuda::smoothing_factor(
            frames_[i].createTexture<float>(Channel::Depth),
            frames_[i].createTexture<float>(Channel::Energy, ftl::rgbd::Format<float>(frames_[i].get<GpuMat>(Channel::Depth).size())),
            frames_[i].createTexture<float>(Channel::Smoothing, ftl::rgbd::Format<float>(frames_[i].get<GpuMat>(Channel::Depth).size())),
            var_thresh,
            s->parameters(), 0
        );

        cv::cuda::resize(frames_[i].get<GpuMat>(Channel::Smoothing), temp_, f.get<cv::cuda::GpuMat>(Channel::Depth).size(), 0.0, 0.0, cv::INTER_LINEAR);
        cv::cuda::add(temp_, f.get<GpuMat>(Channel::Smoothing), f.get<GpuMat>(Channel::Smoothing));
    }*/

    //cv::cuda::subtract(f.get<GpuMat>(Channel::Depth), f.get<GpuMat>(Channel::Smoothing), f.get<GpuMat>(Channel::Depth));

	return true;
}

// ====== Smoothing Channel ====================================================

SmoothChannel::SmoothChannel(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

SmoothChannel::~SmoothChannel() {

}

bool SmoothChannel::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	int radius = config()->value("radius", 3);
	float threshold = config()->value("threshold", 30.0f);
	int iters = max(0, min(6, config()->value("levels", 4)));

	int width = s->parameters().width;
	int height = s->parameters().height;
	float scale = 1.0f;

	// Clear to max smoothing
	out.create<GpuMat>(Channel::Smoothing, Format<float>(width, height)).setTo(cv::Scalar(1.0f));

	// Reduce to nearest
	ftl::cuda::smooth_channel(
		in.createTexture<uchar4>(Channel::Colour),
		//in.createTexture<float>(Channel::Depth),
		out.createTexture<float>(Channel::Smoothing),
		s->parameters(),
		threshold,
		scale,
		radius,
		stream
	);

	for (int i=0; i<iters; ++i) {
		width /= 2;
		height /= 2;
		scale *= 2.0f;

		ftl::rgbd::Camera scaledCam = s->parameters().scaled(width, height);

		// Downscale images for next pass
		cv::cuda::resize(in.get<GpuMat>(Channel::Colour), temp_[i].create<GpuMat>(Channel::Colour), cv::Size(width, height), 0.0, 0.0, cv::INTER_LINEAR);
		//cv::cuda::resize(in.get<GpuMat>(Channel::Depth), temp_[i].create<GpuMat>(Channel::Depth), cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);

		ftl::cuda::smooth_channel(
			temp_[i].createTexture<uchar4>(Channel::Colour),
			//temp_[i].createTexture<float>(Channel::Depth),
			out.getTexture<float>(Channel::Smoothing),
			scaledCam,
			threshold,
			scale,
			radius,
			stream
		);
	}

	return true;
}



// ===== MLS ===================================================================

SimpleMLS::SimpleMLS(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

SimpleMLS::~SimpleMLS() {

}

bool SimpleMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	float thresh = config()->value("mls_threshold", 0.04f);
	int iters = config()->value("mls_iterations", 1);
	int radius = config()->value("mls_radius",2);

	if (!in.hasChannel(Channel::Normals)) {
		/*ftl::cuda::normals(
			in.createTexture<float4>(Channel::Normals, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Depth),
			s->parameters(), 0
		);*/
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {
		ftl::cuda::mls_smooth(
			in.createTexture<float4>(Channel::Normals),
			in.createTexture<float4>(Channel::Points, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Depth),
			in.createTexture<float>(Channel::Depth2, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			thresh,
			radius,
			s->parameters(),
			0
		);

		in.swapChannels(Channel::Depth, Channel::Depth2);
		in.swapChannels(Channel::Normals, Channel::Points);
	}

	return true;
}



ColourMLS::ColourMLS(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

ColourMLS::~ColourMLS() {

}

bool ColourMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	float thresh = config()->value("mls_threshold", 0.04f);
	float col_smooth = config()->value("mls_colour_smoothing", 30.0f);
	int iters = config()->value("mls_iterations", 10);
	int radius = config()->value("mls_radius",3);
	bool crosssup = config()->value("cross_support", false);
	bool filling = config()->value("filling", false);

	if (!in.hasChannel(Channel::Normals)) {
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {
		if (!crosssup) {
			ftl::cuda::colour_mls_smooth(
				in.createTexture<float4>(Channel::Normals),
				in.createTexture<float4>(Channel::Points, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::Depth2, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<uchar4>(Channel::Colour),
				thresh,
				col_smooth,
				radius,
				s->parameters(),
				0
			);
		} else {
			ftl::cuda::colour_mls_smooth_csr(
				in.createTexture<uchar4>(Channel::Support1),
				in.createTexture<float4>(Channel::Normals),
				in.createTexture<float4>(Channel::Points, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::Depth2, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<uchar4>(Channel::Colour),
				thresh,
				col_smooth,
				filling,
				s->parameters(),
				0
			);
		}

		in.swapChannels(Channel::Depth, Channel::Depth2);
		in.swapChannels(Channel::Normals, Channel::Points);
	}

	return true;
}


// ====== Adaptive MLS =========================================================

AdaptiveMLS::AdaptiveMLS(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

AdaptiveMLS::~AdaptiveMLS() {

}

bool AdaptiveMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	int iters = config()->value("mls_iterations", 1);
	int radius = config()->value("mls_radius",2);

	if (!in.hasChannel(Channel::Normals)) {
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {
		ftl::cuda::adaptive_mls_smooth(
			in.createTexture<float4>(Channel::Normals),
			in.createTexture<float4>(Channel::Points, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Depth),
			in.createTexture<float>(Channel::Depth2, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Smoothing),
			radius,
			s->parameters(),
			0
		);

		in.swapChannels(Channel::Depth, Channel::Depth2);
		in.swapChannels(Channel::Normals, Channel::Points);
	}

	return true;
}

