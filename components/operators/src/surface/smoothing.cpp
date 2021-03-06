#include <ftl/operators/smoothing.hpp>
#include <ftl/operators/cuda/smoothing_cuda.hpp>
#include <ftl/operators/cuda/mls_cuda.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <ftl/cuda/normals.hpp>

#include <opencv2/cudawarping.hpp>

using ftl::operators::HFSmoother;
using ftl::operators::SimpleMLS;
using ftl::operators::ColourMLS;
using ftl::operators::AggreMLS;
using ftl::operators::AdaptiveMLS;
using ftl::operators::SmoothChannel;
using ftl::codecs::Channel;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

HFSmoother::HFSmoother(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

HFSmoother::~HFSmoother() {

}

bool HFSmoother::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
    float var_thresh = config()->value("variance_threshold", 0.0002f);
    //int levels = max(0, min(config()->value("levels",0), 4));
    int iters = config()->value("iterations",5);

	if (!in.hasChannel(Channel::Depth)) return false;

	// FIXME: in and out are assumed to be the same

    for (int i=0; i<iters; ++i) {
        ftl::cuda::smoothing_factor(
            in.createTexture<float>(Channel::Depth),
            in.createTexture<float>(Channel::Energy, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            in.createTexture<float>(Channel::Smoothing, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            var_thresh,
            in.getLeftCamera(), stream
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

SmoothChannel::SmoothChannel(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

SmoothChannel::~SmoothChannel() {

}

bool SmoothChannel::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	int radius = config()->value("radius", 3);
	float threshold = config()->value("threshold", 30.0f);
	int iters = max(0, min(6, config()->value("levels", 4)));

	int width = in.getLeftCamera().width;
	int height = in.getLeftCamera().height;
	float scale = 1.0f;

	// Clear to max smoothing
	out.create<ftl::rgbd::VideoFrame>(Channel::Smoothing).createGPU(Format<float>(width, height)).setTo(cv::Scalar(1.0f));

	// Reduce to nearest
	ftl::cuda::smooth_channel(
		in.createTexture<uchar4>(Channel::Colour),
		//in.createTexture<float>(Channel::Depth),
		out.createTexture<float>(Channel::Smoothing),
		in.getLeftCamera(),
		threshold,
		scale,
		radius,
		stream
	);

	for (int i=0; i<iters; ++i) {
		width /= 2;
		height /= 2;
		scale *= 2.0f;

		temp_[i].create(width,height);

		ftl::rgbd::Camera scaledCam = in.getLeftCamera().scaled(width, height);

		// Downscale images for next pass
		cv::cuda::resize(in.get<GpuMat>(Channel::Colour), temp_[i].to_gpumat(), cv::Size(width, height), 0.0, 0.0, cv::INTER_LINEAR);
		//cv::cuda::resize(in.get<GpuMat>(Channel::Depth), temp_[i].create<GpuMat>(Channel::Depth), cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);

		ftl::cuda::smooth_channel(
			temp_[i],
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

SimpleMLS::SimpleMLS(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg), temp_(ftl::data::Frame::make_standalone()) {

}

SimpleMLS::~SimpleMLS() {

}

bool SimpleMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	float thresh = config()->value("mls_threshold", 0.04f);
	int iters = config()->value("mls_iterations", 1);
	int radius = config()->value("mls_radius",2);

	auto &temp = temp_.cast<ftl::rgbd::Frame>();

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
			in.get<GpuMat>(Channel::Normals),
			temp.create<GpuMat>(Channel::Normals),
			in.get<GpuMat>(Channel::Depth),
			temp.create<GpuMat>(Channel::Depth),
			thresh,
			radius,
			in.getLeftCamera(),
			stream
		);

		//in.swapChannels(Channel::Depth, Channel::Depth2);
		//in.swapChannels(Channel::Normals, Channel::Points);
		temp.swapChannel(Channel::Normals, in);
		temp.swapChannel(Channel::Depth, in);
	}

	return true;
}



ColourMLS::ColourMLS(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg), temp_(ftl::data::Frame::make_standalone()) {

}

ColourMLS::~ColourMLS() {

}

bool ColourMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	float thresh = config()->value("mls_threshold", 0.4f);
	float col_smooth = config()->value("mls_colour_smoothing", 30.0f);
	int iters = config()->value("mls_iterations", 3);
	int radius = config()->value("mls_radius",3);
	bool crosssup = config()->value("cross_support", true);
	bool filling = config()->value("filling", false);

	if (!in.hasChannel(Channel::Normals)) {
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	auto &temp = temp_.cast<ftl::rgbd::Frame>();
	auto size = in.get<GpuMat>(Channel::Depth).size();

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	const GpuMat &rgb = in.get<GpuMat>(Channel::Colour);
	GpuMat rgb_buf;
	if (rgb.size() != size) {
		if (graph()->hasBuffer(Buffer::LowLeft, in.source())) {
			rgb_buf = graph()->getBuffer(Buffer::LowLeft, in.source());
		} else {
			auto &t = graph()->createBuffer(Buffer::LowLeft, in.source());
			cv::cuda::resize(rgb, t, size, 0, 0, cv::INTER_LINEAR, cvstream);
			rgb_buf = t;
		}
	} else {
		rgb_buf = rgb;
	}

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {
		if (!crosssup) {
			ftl::cuda::colour_mls_smooth(
				in.createTexture<half4>(Channel::Normals),
				temp.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<float>(Channel::Depth),
				temp.createTexture<float>(Channel::Depth, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<uchar4>(Channel::Colour),
				thresh,
				col_smooth,
				radius,
				in.getLeftCamera(),
				stream
			);
		} else {
			ftl::cuda::colour_mls_smooth_csr(
				in.createTexture<uchar4>(Channel::Support1),
				in.createTexture<half4>(Channel::Normals),
				temp.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<float>(Channel::Depth),
				temp.createTexture<float>(Channel::Depth, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				//in.createTexture<uchar4>(Channel::Colour),
				rgb_buf,
				thresh,
				col_smooth,
				filling,
				in.getLeftCamera(),
				stream
			);
		}

		//in.swapChannels(Channel::Depth, Channel::Depth2);
		//in.swapChannels(Channel::Normals, Channel::Points);
		temp_.swapChannel(Channel::Depth, in);
		temp_.swapChannel(Channel::Normals, in);
	}

	return true;
}


// ====== Aggregating MLS ======================================================

AggreMLS::AggreMLS(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg), temp_(ftl::data::Frame::make_standalone()) {
	temp_.store();
}

AggreMLS::~AggreMLS() {

}

bool AggreMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	float thresh = config()->value("mls_threshold", 0.4f);
	float col_smooth = config()->value("mls_colour_smoothing", 30.0f);
	int iters = config()->value("mls_iterations", 3);
	int radius = config()->value("mls_radius",5);
	bool aggre = config()->value("aggregation", true);

	if (!in.hasChannel(Channel::Normals)) {
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	if (!in.hasChannel(Channel::Support1)) {
		LOG(ERROR) << "Required support channel missing for MLS";
		return false;
	}

	auto &temp = temp_.cast<ftl::rgbd::Frame>();

	auto size = in.get<GpuMat>(Channel::Depth).size();
	centroid_horiz_.create(size.height, size.width);
	normals_horiz_.create(size.height, size.width);
	centroid_vert_.create(size.width, size.height);

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	const GpuMat &rgb = in.get<GpuMat>(Channel::Colour);
	GpuMat rgb_buf;
	if (rgb.size() != size) {
		if (graph()->hasBuffer(Buffer::LowLeft, in.source())) {
			rgb_buf = graph()->getBuffer(Buffer::LowLeft, in.source());
		} else {
			auto &t = graph()->createBuffer(Buffer::LowLeft, in.source());
			cv::cuda::resize(rgb, t, size, 0, 0, cv::INTER_LINEAR, cvstream);
			rgb_buf = t;
		}
	} else {
		rgb_buf = rgb;
	}

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {

		if (aggre) {
			ftl::cuda::mls_aggr_horiz(
				in.createTexture<uchar4>(Channel::Support1),
				in.createTexture<half4>(Channel::Normals),
				normals_horiz_,
				in.createTexture<float>(Channel::Depth),
				centroid_horiz_,
				//in.createTexture<uchar4>(Channel::Colour),
				rgb_buf,
				thresh,
				col_smooth,
				radius,
				in.getLeftCamera(),
				stream
			);

			ftl::cuda::mls_aggr_vert(
				in.createTexture<uchar4>(Channel::Support1),
				normals_horiz_,
				in.createTexture<half4>(Channel::Normals),
				centroid_horiz_,
				centroid_vert_,
				thresh,
				col_smooth,
				radius,
				in.getLeftCamera(),
				stream
			);

			ftl::cuda::mls_adjust_depth(
				in.createTexture<half4>(Channel::Normals),
				centroid_vert_,
				temp.createTexture<float>(Channel::Depth, ftl::rgbd::Format<float>(size)),
				in.getTexture<float>(Channel::Depth),
				in.getLeftCamera(),
				stream
			);

			//in.swapChannels(Channel::Depth, Channel::Depth2);
			//in.swapChannels(Channel::Normals, Channel::Points);
			temp_.swapChannel(Channel::Depth, in);

		} else {
			ftl::cuda::colour_mls_smooth_csr(
				in.createTexture<uchar4>(Channel::Support1),
				in.createTexture<half4>(Channel::Normals),
				temp.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				in.createTexture<float>(Channel::Depth),
				temp.createTexture<float>(Channel::Depth, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
				//in.createTexture<uchar4>(Channel::Colour),
				rgb_buf,
				thresh,
				col_smooth,
				false,
				in.getLeftCamera(),
				stream
			);

			temp_.swapChannel(Channel::Depth, in);
			temp_.swapChannel(Channel::Normals, in);
		}
	}

	return true;
}


// ====== Adaptive MLS =========================================================

AdaptiveMLS::AdaptiveMLS(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg), temp_(ftl::data::Frame::make_standalone()) {

}

AdaptiveMLS::~AdaptiveMLS() {

}

bool AdaptiveMLS::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	int iters = config()->value("mls_iterations", 1);
	int radius = config()->value("mls_radius",2);

	if (!in.hasChannel(Channel::Normals)) {
		LOG(ERROR) << "Required normals channel missing for MLS";
		return false;
	}

	auto &temp = temp_.cast<ftl::rgbd::Frame>();

	// FIXME: Assume in and out are the same frame.
	for (int i=0; i<iters; ++i) {
		ftl::cuda::adaptive_mls_smooth(
			in.createTexture<half4>(Channel::Normals),
			temp.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Depth),
			temp.createTexture<float>(Channel::Depth, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
			in.createTexture<float>(Channel::Smoothing),
			radius,
			in.getLeftCamera(),
			stream
		);

		temp_.swapChannel(Channel::Depth, in);
		temp_.swapChannel(Channel::Normals, in);

	}

	return true;
}

