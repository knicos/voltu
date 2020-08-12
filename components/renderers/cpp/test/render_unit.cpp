#include "catch.hpp"

#include <ftl/data/new_frameset.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/render/CUDARender.hpp>

#include <nlohmann/json.hpp>

using ftl::data::Frame;
using ftl::data::FrameSet;
using ftl::config::json_t;
using ftl::codecs::Channel;

TEST_CASE("Renderer Single Frame", "") {
	json_t global = json_t{{"$id","ftl://test"}};
	auto *root = ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
	f.store();
	auto fsptr = FrameSet::fromFrame(f);

	auto renderer = std::unique_ptr<ftl::render::CUDARender>(
		ftl::create<ftl::render::CUDARender>(root, "renderer")
	);

	Frame out = pool.allocate(ftl::data::FrameID(1,0), 1000);
	out.store();

	ftl::rgbd::Frame &rgbdframe = out.cast<ftl::rgbd::Frame>();
	auto &calib = rgbdframe.setLeft();
	calib.width = 640;
	calib.height = 480;
	calib.fx = 700;
	calib.fy = 700;
	calib.cx = -250;
	calib.cy = -200;
	calib.minDepth = 0.1f;
	calib.maxDepth = 10.0f;
	rgbdframe.setPose() = Eigen::Matrix4d::Identity();

	int width = rgbdframe.getLeft().width;
	int height = rgbdframe.getLeft().height;
	
	auto &colour = rgbdframe.create<cv::cuda::GpuMat>(Channel::Colour);
	colour.create(height, width, CV_8UC4);
	rgbdframe.create<cv::cuda::GpuMat>(Channel::Depth).create(height, width, CV_32F);
	rgbdframe.createTexture<float>(Channel::Depth);

	SECTION("copes with single frame missing colour") {
		for (int i=0; i<20; ++i) {
			renderer->begin(out.cast<ftl::rgbd::Frame>(), Channel::Colour);

			Eigen::Matrix4d pose;
			pose.setIdentity();
			renderer->submit(fsptr.get(), ftl::codecs::Channels<0>(Channel::Colour), pose);
			renderer->render();
			renderer->end();
		}
	}

	/*SECTION("single colour empty mat") {
		fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour);
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setLeft() = calib;
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setPose() = Eigen::Matrix4d::Identity();

		for (int i=0; i<20; ++i) {
			renderer->begin(out.cast<ftl::rgbd::Frame>(), Channel::Colour);

			Eigen::Matrix4d pose;
			pose.setIdentity();
			renderer->submit(fsptr.get(), ftl::codecs::Channels<0>(Channel::Colour), pose);
			renderer->render();
			renderer->end();
		}
	}*/

	SECTION("single colour only frame") {
		fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(640,480,CV_8UC4);
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setLeft() = calib;
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setPose() = Eigen::Matrix4d::Identity();

		for (int i=0; i<20; ++i) {
			renderer->begin(out.cast<ftl::rgbd::Frame>(), Channel::Colour);

			Eigen::Matrix4d pose;
			pose.setIdentity();
			renderer->submit(fsptr.get(), ftl::codecs::Channels<0>(Channel::Colour), pose);
			renderer->render();
			renderer->end();
		}
	}

	SECTION("single full only frame") {
		fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(640,480,CV_8UC4);
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setLeft() = calib;
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setPose() = Eigen::Matrix4d::Identity();

		auto &depth = fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour);
		depth.create(640,480,CV_8UC4);
		depth.setTo(cv::Scalar(5.0f));

		for (int i=0; i<20; ++i) {
			renderer->begin(out.cast<ftl::rgbd::Frame>(), Channel::Colour);

			Eigen::Matrix4d pose;
			pose.setIdentity();
			renderer->submit(fsptr.get(), ftl::codecs::Channels<0>(Channel::Colour), pose);
			renderer->render();
			renderer->end();
		}
	}

	SECTION("single frame empty depth") {
		fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(640,480,CV_8UC4);
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setLeft() = calib;
		fsptr->frames[0].cast<ftl::rgbd::Frame>().setPose() = Eigen::Matrix4d::Identity();

		auto &depth = fsptr->frames[0].create<cv::cuda::GpuMat>(Channel::Colour);
		//depth.create(640,480,CV_8UC4);
		//depth.setTo(cv::Scalar(5.0f));

		for (int i=0; i<20; ++i) {
			renderer->begin(out.cast<ftl::rgbd::Frame>(), Channel::Colour);

			Eigen::Matrix4d pose;
			pose.setIdentity();
			renderer->submit(fsptr.get(), ftl::codecs::Channels<0>(Channel::Colour), pose);
			renderer->render();
			renderer->end();
		}
	}
}
