#include "camera.hpp"
#include "pose_window.hpp"
#include "screen.hpp"
#include <nanogui/glutil.h>

#include <ftl/profiler.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/cudaarithm.hpp>

#include <ftl/operators/antialiasing.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/render/colouriser.hpp>
#include <ftl/cuda/transform.hpp>
#include <ftl/operators/gt_analysis.hpp>
#include <ftl/cuda/colour_cuda.hpp>

#include <ftl/render/overlay.hpp>
#include "statsimage.hpp"

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <fstream>

#ifdef HAVE_OPENVR
#include "vr.hpp"
#endif

using ftl::rgbd::isValidDepth;
using ftl::gui::GLTexture;
using ftl::gui::PoseWindow;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using cv::cuda::GpuMat;


static int vcamcount = 0;

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
	Eigen::Affine3d rx =
		Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
	Eigen::Affine3d ry =
		Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
	Eigen::Affine3d rz =
		Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
	return rz * rx * ry;
}

ftl::gui::Camera::Camera(ftl::gui::Screen *screen, int fsmask, int fid, ftl::codecs::Channel c)
		: screen_(screen), fsmask_(fsmask), fid_(fid), texture1_(GLTexture::Type::BGRA), texture2_(GLTexture::Type::BGRA), depth1_(GLTexture::Type::Float), channel_(c),channels_(0u) {

	eye_ = Eigen::Vector3d::Zero();
	neye_ = Eigen::Vector4d::Zero();
	rotmat_.setIdentity();

	//up_ = Eigen::Vector3f(0,1.0f,0);
	lerpSpeed_ = 0.999f;
	sdepth_ = false;
	ftime_ = (float)glfwGetTime();
	pause_ = false;

#ifdef HAVE_OPENVR
	vr_mode_ = false;
#endif

	//channel_ = Channel::Left;

	channels_ += c;
	//channels_ += Channel::Depth;
	width_ = 0;
	height_ = 0;

	// Create pose window...
	//posewin_ = new PoseWindow(screen, src_->getURI());
	//posewin_->setTheme(screen->windowtheme);
	//posewin_->setVisible(false);
	posewin_ = nullptr;
	renderer_ = nullptr;
	renderer2_ = nullptr;
	post_pipe_ = nullptr;
	record_stream_ = nullptr;
	transform_ix_ = -1;
	stereo_ = false;
	rx_ = 0;
	ry_ = 0;
	framesets_ = nullptr;

	colouriser_ = ftl::create<ftl::render::Colouriser>(screen->root(), "colouriser");
	overlayer_ = ftl::create<ftl::overlay::Overlay>(screen->root(), "overlay");

	// Is virtual camera?
	if (fid == 255) {
		renderer_ = ftl::create<ftl::render::CUDARender>(screen_->root(), std::string("vcam")+std::to_string(vcamcount++));
		// Allow mask to be changed
		fsmask_ = renderer_->value("fsmask", fsmask_);
		renderer_->on("fsmask", [this](const ftl::config::Event &e) {
			fsmask_ = renderer_->value("fsmask", fsmask_);
		});

		intrinsics_ = ftl::create<ftl::Configurable>(renderer_, "intrinsics");

		state_.getLeft() = ftl::rgbd::Camera::from(intrinsics_);
		state_.getRight() = state_.getLeft();

		{
			Eigen::Matrix4d pose;
			pose.setIdentity();
			state_.setPose(pose);

			for (auto &t : transforms_) {
				t.setIdentity();
			}
		}
		{
			double camera_initial_x = intrinsics_->value("camera_x", 0.0);
			double camera_initial_y = intrinsics_->value("camera_y", -1.75);
			double camera_initial_z = intrinsics_->value("camera_z", 0.0);

			double lookat_initial_x = intrinsics_->value("lookat_x", 1.0);
			double lookat_initial_y = intrinsics_->value("lookat_y", 0.0);
			double lookat_initial_z = intrinsics_->value("lookat_z", 0.0);

			Eigen::Vector3f head(camera_initial_x, camera_initial_y, camera_initial_z);
			Eigen::Vector3f lookat(lookat_initial_x, lookat_initial_y, lookat_initial_z);
			// TODO up vector
			Eigen::Matrix4f pose = nanogui::lookAt(head, head+lookat, Eigen::Vector3f(0.0f, 1.0f, 0.0f));

			eye_ = Eigen::Vector3d(camera_initial_x, camera_initial_y, camera_initial_z);
			neye_ = Eigen::Vector4d(eye_(0), eye_(1), eye_(2), 0.0);
			rotmat_ = pose.cast<double>();
			rotmat_.block(0, 3, 3, 1).setZero();
		}
	}
}

ftl::gui::Camera::~Camera() {
	//delete writer_;
	//delete fileout_;
}

void ftl::gui::Camera::drawUpdated(std::vector<ftl::rgbd::FrameSet*> &fss) {
	// Only draw if frameset updated.
	if (!stale_frame_.test_and_set()) {
		draw(fss);
	}
}

void ftl::gui::Camera::draw(std::vector<ftl::rgbd::FrameSet*> &fss) {
	if (fid_ != 255) {
		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;
			UNIQUE_LOCK(fs->mtx, lk);

			ftl::rgbd::Frame *frame = nullptr;

			if ((size_t)fid_ >= fs->frames.size()) return;
			if (!fs->hasFrame(fid_)) return;

			frame = &fs->frames[fid_];
			if (!frame->hasChannel(channel_)) return;

			auto &buf = colouriser_->colourise(*frame, channel_, 0);
			auto &buf2 = frame->getTexture<uchar4>(Channel::Colour);
			ftl::cuda::compositeInverse(buf2, buf, 0);

			// For non-virtual cameras, copy the CUDA texture into the opengl
			// texture device-to-device.
			texture1_.make(buf.width(), buf.height());
			auto dst1 = texture1_.map(0);
			cudaMemcpy2D(dst1.data, dst1.step1(), buf.devicePtr(), buf.pitch(), buf.width()*4, buf.height(), cudaMemcpyDeviceToDevice);
			ftl::cuda::flip<uchar4>(dst1, 0);
			texture1_.unmap(0);

			depth1_.make(buf.width(), buf.height());
			dst1 = depth1_.map(0);
			dst1.setTo(cv::Scalar(0.5f));
			depth1_.unmap(0);

			width_ = texture1_.width();
			height_ = texture1_.height();
			return;
		}
	}
	//if (fsid_ >= fss.size()) return;

	//auto &fs = *fss[fsid_];

	_applyPoseEffects(fss);

	UNIQUE_LOCK(mutex_, lk2);
	//state_.getLeft().fx = intrinsics_->value("focal", 700.0f);
	//state_.getLeft().fy = state_.getLeft().fx;
	_draw(fss);
}

std::pair<const ftl::rgbd::Frame *, const ftl::codecs::Face *> ftl::gui::Camera::_selectFace(std::vector<ftl::rgbd::FrameSet*> &fss) {
	for (auto *fset : fss) {
		for (const auto &f : fset->frames) {
			if (f.hasChannel(Channel::Faces)) {
				std::vector<ftl::codecs::Face> data;
				f.get(Channel::Faces, data);

				if (data.size() > 0) {
					return {&f,&(*data.rbegin())};
				}
			}
		}
	}
	return {nullptr, nullptr};
}

void ftl::gui::Camera::_generateWindow(const ftl::rgbd::Frame &f, const ftl::codecs::Face &face, Eigen::Matrix4d &pose_adjust, ftl::render::ViewPort &vp) {
	auto cam = ftl::rgbd::Camera::from(intrinsics_);
	auto d = face;

	float screenWidth = intrinsics_->value("screen_size", 0.6f);  // In meters
	float screenHeight = (9.0f/16.0f) * screenWidth;

	float screenDistance = (d.depth > cam.minDepth && d.depth < cam.maxDepth) ? d.depth : intrinsics_->value("screen_dist_default", 0.5f);  // Face distance from screen in meters

	auto pos = f.getLeft().screenToCam(float(d.box.x+(d.box.width/2)), float(d.box.y+(d.box.height/2)), screenDistance);
	Eigen::Vector3f eye;
	eye[0] = -pos.x;
	eye[1] = pos.y;
	eye[2] = -pos.z;
	//eye[3] = 0;

	Eigen::Translation3f trans(eye);
	Eigen::Affine3f t(trans);
	Eigen::Matrix4f viewPose = t.matrix();

	// Calculate where the screen is within current camera space
	Eigen::Vector4f p1 = viewPose.cast<float>() * (Eigen::Vector4f(screenWidth/2.0, screenHeight/2.0, 0, 1));
	Eigen::Vector4f p2 = viewPose.cast<float>() * (Eigen::Vector4f(screenWidth/2.0, -screenHeight/2.0, 0, 1));
	Eigen::Vector4f p3 = viewPose.cast<float>() * (Eigen::Vector4f(-screenWidth/2.0, screenHeight/2.0, 0, 1));
	Eigen::Vector4f p4 = viewPose.cast<float>() * (Eigen::Vector4f(-screenWidth/2.0, -screenHeight/2.0, 0, 1));
	p1 = p1 / p1[3];
	p2 = p2 / p2[3];
	p3 = p3 / p3[3];
	p4 = p4 / p4[3];
	float2 p1screen = cam.camToScreen<float2>(make_float3(p1[0],p1[1],p1[2]));
	float2 p2screen = cam.camToScreen<float2>(make_float3(p2[0],p2[1],p2[2]));
	float2 p3screen = cam.camToScreen<float2>(make_float3(p3[0],p3[1],p3[2]));
	float2 p4screen = cam.camToScreen<float2>(make_float3(p4[0],p4[1],p4[2]));

	std::vector<cv::Point2f> quad_pts;
	std::vector<cv::Point2f> squre_pts;
	quad_pts.push_back(cv::Point2f(p1screen.x,p1screen.y));
	quad_pts.push_back(cv::Point2f(p2screen.x,p2screen.y));
	quad_pts.push_back(cv::Point2f(p3screen.x,p3screen.y));
	quad_pts.push_back(cv::Point2f(p4screen.x,p4screen.y));
	squre_pts.push_back(cv::Point2f(0,0));
	squre_pts.push_back(cv::Point2f(0,cam.height));
	squre_pts.push_back(cv::Point2f(cam.width,0));
	squre_pts.push_back(cv::Point2f(cam.width,cam.height));

	cv::Mat transmtx = cv::getPerspectiveTransform(quad_pts,squre_pts);
	//cv::Mat transformed = cv::Mat::zeros(overlay_.rows, overlay_.cols, CV_8UC4);
	//cv::warpPerspective(im1_, im1_, transmtx, im1_.size());

	// TODO: Use the transmtx above for perspective distortion..

	//ftl::render::ViewPort vp;
	vp.x = std::min(p4screen.x, std::min(p3screen.x, std::min(p1screen.x,p2screen.x)));
	vp.y = std::min(p4screen.y, std::min(p3screen.y, std::min(p1screen.y,p2screen.y)));
	vp.width = std::max(p4screen.x, std::max(p3screen.x, std::max(p1screen.x,p2screen.x))) - vp.x;
	vp.height = std::max(p4screen.y, std::max(p3screen.y, std::max(p1screen.y,p2screen.y))) - vp.y;
	/*vp.warpMatrix.entries[0] = transmtx.at<float>(0,0);
	vp.warpMatrix.entries[1] = transmtx.at<float>(1,0);
	vp.warpMatrix.entries[2] = transmtx.at<float>(2,0);
	vp.warpMatrix.entries[3] = transmtx.at<float>(0,1);
	vp.warpMatrix.entries[4] = transmtx.at<float>(1,1);
	vp.warpMatrix.entries[5] = transmtx.at<float>(2,1);
	vp.warpMatrix.entries[6] = transmtx.at<float>(0,2);
	vp.warpMatrix.entries[7] = transmtx.at<float>(1,2);
	vp.warpMatrix.entries[8] = transmtx.at<float>(2,2);
	vp.warpMatrix = vp.warpMatrix.getInverse(); //.getInverse();*/
	//renderer_->setViewPort(ftl::render::ViewPortMode::Warping, vp);

	pose_adjust = viewPose.cast<double>();
}

void ftl::gui::Camera::_applyPoseEffects(std::vector<ftl::rgbd::FrameSet*> &fss) {
	if (renderer_->value("window_effect", false)) {
		auto [frame,face] = _selectFace(fss);
		if (face) {
			Eigen::Matrix4d windowPose;
			ftl::render::ViewPort windowViewPort;
			_generateWindow(*frame, *face, windowPose, windowViewPort);

			// Apply the window effect
			renderer_->setViewPort(ftl::render::ViewPortMode::Stretch, windowViewPort);
			state_.getPose() = windowPose * state_.getPose();
		}
	}
}

void ftl::gui::Camera::setStereo(bool v) {
	UNIQUE_LOCK(mutex_, lk);

	if (isVirtual()) {
		stereo_ = v;
	} else if (v && availableChannels().has(Channel::Right)) {
		stereo_ = true;
	} else {
		stereo_ = false;
	}
}

static ftl::codecs::Channel mapToSecondChannel(ftl::codecs::Channel c) {
	switch (c) {
		case Channel::Depth		: return Channel::Depth2;
		case Channel::Normals	: return Channel::Normals2;
		default: return c;
	}
}

void ftl::gui::Camera::_draw(std::vector<ftl::rgbd::FrameSet*> &fss) {
	frame_.reset();
	frame_.setOrigin(&state_);

	// Make sure an OpenGL pixel buffer exists
	texture1_.make(state_.getLeft().width, state_.getLeft().height);
	depth1_.make(state_.getLeft().width, state_.getLeft().height);
	if (isStereo()) texture2_.make(state_.getRight().width, state_.getRight().height);

	// Map the GL pixel buffer to a GpuMat
	frame_.create<cv::cuda::GpuMat>(Channel::Colour) = texture1_.map(renderer_->getCUDAStream());
	frame_.create<cv::cuda::GpuMat>(Channel::Depth) = depth1_.map(renderer_->getCUDAStream());
	frame_.createTexture<float>(Channel::Depth);
	if (isStereo()) frame_.create<cv::cuda::GpuMat>(Channel::Colour2) = texture2_.map((renderer2_) ? renderer2_->getCUDAStream() : 0);

	// TODO: Remove;
	overlay_.create(state_.getLeft().height, state_.getLeft().width, CV_8UC4);
	//frame_.create<cv::Mat>(Channel::Overlay) = overlay_;

	//overlay_.setTo(cv::Scalar(0,0,0,0));
	//bool enable_overlay = overlayer_->value("enabled", false);

	{
		FTL_Profile("Render",0.034);
		renderer_->begin(frame_, Channel::Colour);
		if (isStereo()) {
			if (!renderer2_) {
				renderer2_ = ftl::create<ftl::render::CUDARender>(screen_->root(), std::string("vcam")+std::to_string(vcamcount++));
			}
			renderer2_->begin(frame_, Channel::Colour2);
		}

		try {
			for (auto *fs : fss) {
				if (!usesFrameset(fs->id)) continue;

				fs->mtx.lock();
				renderer_->submit(fs, ftl::codecs::Channels<0>(Channel::Colour), transforms_[fs->id]);
				if (isStereo()) renderer2_->submit(fs, ftl::codecs::Channels<0>(Channel::Colour), transforms_[fs->id]);

				//if (enable_overlay) {
					// Generate and upload an overlay image.
				//	overlayer_->apply(*fs, overlay_, state_);
				//	frame_.upload(Channel::Overlay, renderer_->getCUDAStream());
				//}
			}

			renderer_->render();
			if (isStereo()) renderer2_->render();

			if (channel_ != Channel::Left && channel_ != Channel::Right && channel_ != Channel::None) {
				renderer_->blend(channel_);
				if (isStereo()) {
					renderer2_->blend(mapToSecondChannel(channel_));
				}
			}

			//if (enable_overlay) {
			//	renderer_->blend(Channel::Overlay);
			//}

			renderer_->end();
			if (isStereo()) renderer2_->end();
		} catch(std::exception &e) {
			LOG(ERROR) << "Exception in render: " << e.what();
		}

		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;
			fs->mtx.unlock();
		}
	}

	if (!post_pipe_) {
		post_pipe_ = ftl::config::create<ftl::operators::Graph>(screen_->root(), "post_filters");
		post_pipe_->append<ftl::operators::FXAA>("fxaa");
		post_pipe_->append<ftl::operators::GTAnalysis>("gtanalyse");
	}

	post_pipe_->apply(frame_, frame_, 0);

	channels_ = frame_.getChannels();

	frame_.get<cv::cuda::GpuMat>(Channel::Depth).download(im_depth_);
	cv::flip(im_depth_, im_depth_, 0);

	//frame_.get<cv::cuda::GpuMat>(Channel::Normals).download(im_normals_);
	//im_normals_.createMatHeader().convertTo(im_normals_f_, CV_32FC4);
	//cv::flip(im_normals_f_, im_normals_f_, 0);

	// Normalize depth map
	frame_.get<cv::cuda::GpuMat>(Channel::Depth).convertTo(frame_.get<cv::cuda::GpuMat>(Channel::Depth), CV_32F, 1.0/8.0);

	// Unmap GL buffer from CUDA and finish updating GL texture
	texture1_.unmap(renderer_->getCUDAStream());
	depth1_.unmap(renderer_->getCUDAStream());
	if (isStereo()) texture2_.unmap(renderer2_->getCUDAStream());

	width_ = texture1_.width();
	height_ = texture1_.height();

	if (record_stream_ && record_stream_->active()) {
		// TODO: Allow custom channel selection
		ftl::rgbd::FrameSet fs2;
		auto &f = fs2.frames.emplace_back();
		fs2.count = 1;
		fs2.mask = 1;
		//fs2.stale = false;
		fs2.set(ftl::data::FSFlag::STALE);
		frame_.swapTo(Channels<0>(Channel::Colour), f);  // Channel::Colour + Channel::Depth
		if (f.hasChannel(Channel::Colour)) {
			f.create<cv::cuda::GpuMat>(Channel::Colour2).create(f.get<cv::cuda::GpuMat>(Channel::Colour).size(), f.get<cv::cuda::GpuMat>(Channel::Colour2).type());
			f.swapChannels(Channel::Colour, Channel::Colour2);
			cv::cuda::flip(f.get<cv::cuda::GpuMat>(Channel::Colour2), f.get<cv::cuda::GpuMat>(Channel::Colour), 0);
		}
		fs2.timestamp = ftl::timer::get_time();
		fs2.id = 0;
		record_sender_->post(fs2);
		record_stream_->select(0, Channels<0>(Channel::Colour));
		f.swapTo(Channels<0>(Channel::Colour), frame_);
	} else if (do_snapshot_) {
		do_snapshot_ = false;
		cv::Mat flipped;
		cv::Mat im1;

		frame_.get<cv::cuda::GpuMat>(Channel::Colour).download(im1);

		{
			//UNIQUE_LOCK(mutex_, lk);
			cv::flip(im1, flipped, 0);
		}
		cv::cvtColor(flipped, flipped, cv::COLOR_BGRA2BGR);
		cv::imwrite(snapshot_filename_, flipped);
	}
}

void ftl::gui::Camera::update(int fsid, const ftl::codecs::Channels<0> &c) {
	if (!isVirtual() && ((1 << fsid) & fsmask_)) {
		channels_ += c;
		//if (c.has(Channel::Depth)) {
			//channels_ += Channel::ColourNormals;
		//}
	}
}

void ftl::gui::Camera::update(std::vector<ftl::rgbd::FrameSet*> &fss) {
	UNIQUE_LOCK(mutex_, lk);

	framesets_ = &fss;
	stale_frame_.clear();

	if (screen_->activeCamera() == this) {
		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;

			for (auto &f : fs->frames) {
				//if (f.hasChanged(Channel::Pose)) {
					f.patchPose(T_);
				//}
			}
		}
	}

	//if (fss.size() <= fsid_) return;
	if (fid_ == 255) {
		name_ = "Virtual Camera";
	} else {
		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;

			ftl::rgbd::Frame *frame = nullptr;

			if ((size_t)fid_ >= fs->frames.size()) return;
			frame = &fs->frames[fid_];
			channels_ = frame->getChannels();

			if (frame->hasChannel(Channel::Messages)) {
				msgs_.clear();
				frame->get(Channel::Messages, msgs_);
			}

			auto n = frame->get<std::string>("name");
			if (n) {
				name_ = *n;
			} else {
				name_ = "No name";
			}
			state_.getLeft() = frame->getLeftCamera();
			return;
		}
	}
}

void ftl::gui::Camera::setPose(const Eigen::Matrix4d &p) {
	eye_[0] = p(0,3);
	eye_[1] = p(1,3);
	eye_[2] = p(2,3);

	double sx = Eigen::Vector3d(p(0,0), p(1,0), p(2,0)).norm();
	double sy = Eigen::Vector3d(p(0,1), p(1,1), p(2,1)).norm();
	double sz = Eigen::Vector3d(p(0,2), p(1,2), p(2,2)).norm();

	Eigen::Matrix4d rot = p;
	rot(0,3) = 0.0;
	rot(1,3) = 0.0;
	rot(2,3) = 0.0;
	rot(0,0) = rot(0,0) / sx;
	rot(1,0) = rot(1,0) / sx;
	rot(2,0) = rot(2,0) / sx;
	rot(0,1) = rot(0,1) / sy;
	rot(1,1) = rot(1,1) / sy;
	rot(2,1) = rot(2,1) / sy;
	rot(0,2) = rot(0,2) / sz;
	rot(1,2) = rot(1,2) / sz;
	rot(2,2) = rot(2,2) / sz;
	rotmat_ = rot;
}

void ftl::gui::Camera::mouseMovement(int rx, int ry, int button) {
	//if (!src_->hasCapabilities(ftl::rgbd::kCapMovable)) return;
	if (fid_ < 255) return;
	if (button == 1) {
		rx_ += rx;
		ry_ += ry;

		/*float rrx = ((float)ry * 0.2f * delta_);
		//orientation_[2] += std::cos(orientation_[1])*((float)rel[1] * 0.2f * delta_);
		float rry = (float)rx * 0.2f * delta_;
		float rrz = 0.0;


		Eigen::Affine3d r = create_rotation_matrix(rrx, -rry, rrz);
		rotmat_ = rotmat_ * r.matrix();*/
	}
}

void ftl::gui::Camera::keyMovement(int key, int modifiers) {
	//if (!src_->hasCapabilities(ftl::rgbd::kCapMovable)) return;
	if (fid_ < 255) return;
	if (key == 263 || key == 262) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 263) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(scalar,0.0,0.0,1.0);
		return;
	} else if (key == 264 || key == 265) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 264) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0,0.0,scalar,1.0);
		return;
	} else if (key == 266 || key == 267) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 266) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0,scalar,0.0,1.0);
		return;
	} else if (key >= '0' && key <= '5' && modifiers == 2) {  // Ctrl+NUMBER
		int ix = key - (int)('0');
		transform_ix_ = ix-1;
		return;
	}
}

void ftl::gui::Camera::showPoseWindow() {
	posewin_->setVisible(true);
}

void ftl::gui::Camera::showSettings() {

}

#ifdef HAVE_OPENVR
bool ftl::gui::Camera::setVR(bool on) {
	if (on == vr_mode_) {
		LOG(WARNING) << "VR mode already enabled";
		return on;
	}
	vr_mode_ = on;

	if (on) {
		setStereo(true);

		UNIQUE_LOCK(mutex_, lk);
		//src_->set("baseline", baseline_);
		state_.getLeft().baseline = baseline_;

		Eigen::Matrix3d intrinsic;

		unsigned int size_x, size_y;
		screen_->getVR()->GetRecommendedRenderTargetSize(&size_x, &size_y);
		state_.getLeft().width = size_x;
		state_.getLeft().height = size_y;
		state_.getRight().width = size_x;
		state_.getRight().height = size_y;

		intrinsic = getCameraMatrix(screen_->getVR(), vr::Eye_Left);
		CHECK(intrinsic(0, 2) < 0 && intrinsic(1, 2) < 0);
		state_.getLeft().fx = intrinsic(0,0);
		state_.getLeft().fy = intrinsic(0,0);
		state_.getLeft().cx = intrinsic(0,2);
		state_.getLeft().cy = intrinsic(1,2);

		intrinsic = getCameraMatrix(screen_->getVR(), vr::Eye_Right);
		CHECK(intrinsic(0, 2) < 0 && intrinsic(1, 2) < 0);
		state_.getRight().fx = intrinsic(0,0);
		state_.getRight().fy = intrinsic(0,0);
		state_.getRight().cx = intrinsic(0,2);
		state_.getRight().cy = intrinsic(1,2);

		vr_mode_ = true;
	}
	else {
		vr_mode_ = false;
		setStereo(false);

		UNIQUE_LOCK(mutex_, lk);
		state_.getLeft() = ftl::rgbd::Camera::from(intrinsics_);
		state_.getRight() = state_.getLeft();
	}

	return vr_mode_;
}
#endif

void ftl::gui::Camera::setChannel(Channel c) {
	UNIQUE_LOCK(mutex_, lk);
	channel_ = c;
}

/*static void drawEdges(	const cv::Mat &in, cv::Mat &out,
						const int ksize = 3, double weight = -1.0, const int threshold = 32,
						const int threshold_type = cv::THRESH_TOZERO)
{
	cv::Mat edges;
	cv::Laplacian(in, edges, 8, ksize);
	cv::threshold(edges, edges, threshold, 255, threshold_type);

	cv::Mat edges_color(in.size(), CV_8UC4);
	cv::addWeighted(edges, weight, out, 1.0, 0.0, out, CV_8UC4);
}*/


void ftl::gui::Camera::active(bool a) {
	if (a) {

	} else {
		neye_[0] = eye_[0];
		neye_[1] = eye_[1];
		neye_[2] = eye_[2];
	}
}

void ftl::gui::Camera::drawOverlay(const Eigen::Vector2f &s) {
	if (!framesets_) return;
	//UNIQUE_LOCK(mutex_,lk);
	for (auto *fs : *framesets_) {
		if (!usesFrameset(fs->id)) continue;

		// Generate and upload an overlay image.
		overlayer_->draw(*fs, state_, s);
	}
}

const void ftl::gui::Camera::captureFrame() {
	float now = (float)glfwGetTime();
	if (!screen_->isVR() && (now - ftime_) < 0.04f) return;

	delta_ = now - ftime_;
	ftime_ = now;

	//LOG(INFO) << "Frame delta: " << delta_;

	//if (src_ && src_->isReady()) {
	if (width_ > 0 && height_ > 0) {
		Eigen::Matrix4d viewPose;

		if (screen_->isVR()) {
			#ifdef HAVE_OPENVR

			vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);
			vr::VRCompositor()->WaitGetPoses(rTrackedDevicePose_, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

			if (isStereo() && rTrackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
			{
				Eigen::Matrix4d eye_l = ConvertSteamVRMatrixToMatrix4(
					vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));

				//Eigen::Matrix4d eye_r = ConvertSteamVRMatrixToMatrix4(
				//	vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));

				float baseline_in = 2.0 * eye_l(0, 3);

				if (baseline_in != baseline_) {
					baseline_ = baseline_in;
					//src_->set("baseline", baseline_);
					state_.getLeft().baseline = baseline_;
					state_.getRight().baseline = baseline_;
				}
				Eigen::Matrix4d pose = ConvertSteamVRMatrixToMatrix4(rTrackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
				Eigen::Vector3d ea = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

				Eigen::Vector3d vreye;
				vreye[0] = pose(0, 3);
				vreye[1] = -pose(1, 3);
				vreye[2] = -pose(2, 3);

				// NOTE: If modified, should be verified with VR headset!
				Eigen::Matrix3d R;
				R =		Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) *
						Eigen::AngleAxisd(-ea[1], Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(-ea[2], Eigen::Vector3d::UnitZ());

				//double rd = 180.0 / 3.141592;
				//LOG(INFO) << "rotation x: " << ea[0] *rd << ", y: " << ea[1] * rd << ", z: " << ea[2] * rd;
				// pose.block<3, 3>(0, 0) = R;

				rotmat_.block(0, 0, 3, 3) = R;

				// TODO: Apply a rotation to orient also

				eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
				eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
				eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

				Eigen::Translation3d trans(eye_ + vreye);
				Eigen::Affine3d t(trans);
				viewPose = t.matrix() * rotmat_;

			} else {
				//LOG(ERROR) << "No VR Pose";
			}
			#endif
		} else {
			// Use mouse to move camera

			float rrx = ((float)ry_ * 0.2f * delta_);
			float rry = (float)rx_ * 0.2f * delta_;
			float rrz = 0.0;

			Eigen::Affine3d r = create_rotation_matrix(rrx, -rry, rrz);
			rotmat_ = rotmat_ * r.matrix();

			rx_ = 0;
			ry_ = 0;

			eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
			eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
			eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

			Eigen::Translation3d trans(eye_);
			Eigen::Affine3d t(trans);
			viewPose = t.matrix() * rotmat_;
		}

		{
			UNIQUE_LOCK(mutex_, lk);

			if (isVirtual()) {
				if (transform_ix_ == -1) {
					state_.setPose(viewPose);
				} else if (transform_ix_ >= 0) {
					transforms_[transform_ix_] = viewPose;
				}
			}
		}

		if (framesets_) draw(*framesets_);
	}

	//return texture1_;
}

void ftl::gui::Camera::snapshot(const std::string &filename) {
	/*cv::Mat flipped;

	{
		UNIQUE_LOCK(mutex_, lk);
		//cv::flip(im1_, flipped, 0);
	}
	cv::cvtColor(flipped, flipped, cv::COLOR_BGRA2BGR);
	cv::imwrite(filename, flipped);*/
	snapshot_filename_ = filename;
	do_snapshot_ = true;
}

void ftl::gui::Camera::startVideoRecording(const std::string &filename) {
	if (!record_stream_) {
		record_stream_ = ftl::create<ftl::stream::File>(screen_->root(), "video2d");
		record_stream_->setMode(ftl::stream::File::Mode::Write);
		record_sender_ = ftl::create<ftl::stream::Sender>(screen_->root(), "videoEncode");
		record_sender_->setStream(record_stream_);
		record_sender_->value("codec", 2);  // Default H264
	}

	if (record_stream_->active()) return;

	record_stream_->set("filename", filename);
	record_stream_->begin();
}

void ftl::gui::Camera::stopVideoRecording() {
	if (record_stream_ && record_stream_->active()) record_stream_->end();
}

float ftl::gui::Camera::getDepth(int x, int y) {
	if (x < 0 || y < 0) { return NAN; }
	UNIQUE_LOCK(mutex_, lk);
	if (x >= im_depth_.cols || y >= im_depth_.rows) { return NAN; }
	LOG(INFO) << y << ", " << x;
	return im_depth_.createMatHeader().at<float>(y, x);
}

cv::Point3f ftl::gui::Camera::getPoint(int x, int y) {
	if (x < 0 || y < 0) { return cv::Point3f(); }
	UNIQUE_LOCK(mutex_, lk);
		LOG(INFO) << y << ", " << x;
	if (x >= im_depth_.cols || y >= im_depth_.rows) { return cv::Point3f(); }
	float d = im_depth_.createMatHeader().at<float>(y, x);

	auto point = frame_.getLeftCamera().screenToCam(x, y, d);
	Eigen::Vector4d p(point.x, point.y, point.z, 1.0f);
	Eigen::Matrix4d pose = frame_.getPose();
	Eigen::Vector4d point_eigen = pose * p;
	return cv::Point3f(point_eigen(0), point_eigen(1), point_eigen(2));
}

/*
cv::Point3f ftl::gui::Camera::getNormal(int x, int y) {
	UNIQUE_LOCK(mutex_, lk);
		LOG(INFO) << y << ", " << x;
	if (x >= im_normals_.cols || y >= im_normals_.rows) { return cv::Point3f(); }
	auto n = im_normals_f_.at<cv::Vec4f>(y, x);
	return cv::Point3f(n[0], n[1], n[2]);
}
*/

void ftl::gui::Camera::setTransform(const Eigen::Matrix4d &T) {
	T_ = T * T_;
}

Eigen::Matrix4d ftl::gui::Camera::getTransform() const {
	return T_;
}
