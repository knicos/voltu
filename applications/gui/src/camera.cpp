#include "camera.hpp"
#include "pose_window.hpp"
#include "screen.hpp"
#include <nanogui/glutil.h>

#include <ftl/profiler.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ftl/operators/antialiasing.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/render/colouriser.hpp>

#include <ftl/codecs/faces.hpp>

#include "overlay.hpp"
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

ftl::gui::Camera::Camera(ftl::gui::Screen *screen, int fsmask, int fid, ftl::codecs::Channel c) : screen_(screen), fsmask_(fsmask), fid_(fid), channel_(c),channels_(0u) {
	eye_ = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	neye_ = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
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

		Eigen::Matrix4d pose;
		pose.setIdentity();
		state_.setPose(pose);

		for (auto &t : transforms_) {
			t.setIdentity();
		}
	}
}

ftl::gui::Camera::~Camera() {
	//delete writer_;
	//delete fileout_;
}

static Eigen::Vector3f cudaToEigen(const float3 &v) {
	Eigen::Vector3f e;
	e[0] = v.x;
	e[1] = v.y;
	e[2] = v.z;
	return e;
}

void ftl::gui::Camera::draw(std::vector<ftl::rgbd::FrameSet*> &fss) {
	if (fid_ != 255) return;
	//if (fsid_ >= fss.size()) return;

	//auto &fs = *fss[fsid_];

	UNIQUE_LOCK(mutex_, lk2);
	//state_.getLeft().fx = intrinsics_->value("focal", 700.0f);
	//state_.getLeft().fy = state_.getLeft().fx;
	_draw(fss);

	if (renderer_->value("window_effect", false)) {
		for (auto *fset : fss) {
			for (const auto &f : fset->frames) {
				if (f.hasChannel(Channel::Faces)) {
					std::vector<ftl::codecs::Face> data;
					f.get(Channel::Faces, data);

					if (data.size() > 0) {
						auto &d = *data.rbegin();
						
						cv::Mat over_depth;
						over_depth.create(im1_.size(), CV_32F);

						auto cam = ftl::rgbd::Camera::from(intrinsics_);

						float screenWidth = intrinsics_->value("screen_size", 0.6f);  // In meters
						float screenHeight = (9.0f/16.0f) * screenWidth;

						float screenDistance = (d.depth > cam.minDepth && d.depth < cam.maxDepth) ? d.depth : intrinsics_->value("screen_dist_default", 0.5f);  // Face distance from screen in meters

						auto pos = f.getLeft().screenToCam(d.box.x+(d.box.width/2), d.box.y+(d.box.height/2), screenDistance);
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

						//cv::line(im1_, cv::Point2f(p1screen.x, p1screen.y), cv::Point2f(p2screen.x, p2screen.y), cv::Scalar(255,0,255,0));
						//cv::line(im1_, cv::Point2f(p4screen.x, p4screen.y), cv::Point2f(p2screen.x, p2screen.y), cv::Scalar(255,0,255,0));
						//cv::line(im1_, cv::Point2f(p1screen.x, p1screen.y), cv::Point2f(p3screen.x, p3screen.y), cv::Scalar(255,0,255,0));
						//cv::line(im1_, cv::Point2f(p3screen.x, p3screen.y), cv::Point2f(p4screen.x, p4screen.y), cv::Scalar(255,0,255,0));
						//LOG(INFO) << "DRAW LINE: " << p1screen.x << "," << p1screen.y << " -> " << p2screen.x << "," << p2screen.y;

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
						cv::Mat transformed = cv::Mat::zeros(im1_.rows, im1_.cols, CV_8UC4);
						//cv::warpPerspective(im1_, im1_, transmtx, im1_.size());

						ftl::render::ViewPort vp;
						vp.x = std::min(p4screen.x, std::min(p3screen.x, std::min(p1screen.x,p2screen.x)));
						vp.y = std::min(p4screen.y, std::min(p3screen.y, std::min(p1screen.y,p2screen.y)));
						vp.width = std::max(p4screen.x, std::max(p3screen.x, std::max(p1screen.x,p2screen.x))) - vp.x;
						vp.height = std::max(p4screen.y, std::max(p3screen.y, std::max(p1screen.y,p2screen.y))) - vp.y;
						renderer_->setViewPort(ftl::render::ViewPortMode::Warping, vp);

						//Eigen::Matrix4d windowPose;
						//windowPose.setIdentity();

						//Eigen::Matrix4d fakepose = (viewPose.cast<double>()).inverse() * state_.getPose();
						//ftl::rgbd::Camera fakecam = cam;

						//eye[1] = -eye[1];
						//eye[2] = -eye[2];
						//Eigen::Translation3f trans2(eye);
						//Eigen::Affine3f t2(trans2);
						//Eigen::Matrix4f viewPose2 = t2.matrix();

						// Use face tracked window pose for virtual camera
						//state_.getLeft() = fakecam;
						transform_ix_ = fset->id;  // Disable keyboard/mouse pose setting

						state_.setPose(transforms_[transform_ix_] * viewPose.cast<double>());

						//Eigen::Vector4d pt1 = state_.getPose().inverse() * Eigen::Vector4d(eye[0],eye[1],eye[2],1);
						//pt1 /= pt1[3];
						//Eigen::Vector4d pt2 = state_.getPose().inverse() * Eigen::Vector4d(0,0,0,1);
						//pt2 /= pt2[3];


						//ftl::overlay::draw3DLine(state_.getLeft(), im1_, over_depth, pt1, pt2, cv::Scalar(0,0,255,0));
						//ftl::overlay::drawRectangle(state_.getLeft(), im1_, over_depth, windowPose.inverse() * state_.getPose(), cv::Scalar(255,0,0,0), screenWidth, screenHeight);
						//ftl::overlay::drawCamera(state_.getLeft(), im1_, over_depth, fakecam, fakepose, cv::Scalar(255,0,255,255), 1.0,screen_->root()->value("show_frustrum", false));
					}
				}
			}
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
			}

			if (channel_ != Channel::Left && channel_ != Channel::Right && channel_ != Channel::None) {
				renderer_->blend(0.5f, channel_);
				if (isStereo()) {
					renderer2_->blend(0.5f, mapToSecondChannel(channel_));
				}
			}

			renderer_->end();
			if (isStereo()) renderer2_->end();
		} catch(std::exception &e) {
			LOG(ERROR) << "Exception in render: " << e.what();
		}
		for (auto *fs : fss) {
			fs->mtx.unlock();
		}
	}

	if (!post_pipe_) {
		post_pipe_ = ftl::config::create<ftl::operators::Graph>(screen_->root(), "post_filters");
		post_pipe_->append<ftl::operators::FXAA>("fxaa");
	}

	post_pipe_->apply(frame_, frame_, 0);

	channels_ = frame_.getChannels();

	if (isStereo()) {
		_downloadFrames(frame_.getTexture<uchar4>(Channel::Colour), frame_.getTexture<uchar4>(Channel::Colour2));
	} else {
		_downloadFrame(frame_.getTexture<uchar4>(Channel::Colour));
	}

	if (screen_->root()->value("show_poses", false)) {
		cv::Mat over_col, over_depth;
		over_col.create(im1_.size(), CV_8UC4);
		over_depth.create(im1_.size(), CV_32F);

		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;
			for (size_t i=0; i<fs->frames.size(); ++i) {
				auto pose = fs->frames[i].getPose().inverse() * state_.getPose();
				Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
				pos /= pos[3];

				auto name = fs->frames[i].get<std::string>("name");
				ftl::overlay::drawCamera(state_.getLeft(), im1_, over_depth, fs->frames[i].getLeftCamera(), pose, cv::Scalar(0,0,255,255), 0.2,screen_->root()->value("show_frustrum", false));
				if (name) ftl::overlay::drawText(state_.getLeft(), im1_, over_depth, *name, pos, 0.5, cv::Scalar(0,0,255,255));
			}
		}
	}

	if (record_stream_ && record_stream_->active()) {
		// TODO: Allow custom channel selection
		ftl::rgbd::FrameSet fs2;
		auto &f = fs2.frames.emplace_back();
		fs2.count = 1;
		fs2.mask = 1;
		fs2.stale = false;
		frame_.swapTo(Channels<0>(Channel::Colour), f);  // Channel::Colour + Channel::Depth
		fs2.timestamp = ftl::timer::get_time();
		fs2.id = 0;
		record_sender_->post(fs2);
		record_stream_->select(0, Channels<0>(Channel::Colour));
		f.swapTo(Channels<0>(Channel::Colour), frame_);
	}
}

void ftl::gui::Camera::_downloadFrames(ftl::cuda::TextureObject<uchar4> &a, ftl::cuda::TextureObject<uchar4> &b) {
	im1_.create(cv::Size(a.width(), a.height()), CV_8UC4);
	a.to_gpumat().download(im1_);

	// OpenGL (0,0) bottom left
	cv::flip(im1_, im1_, 0);

	width_ = im1_.cols;
	height_ = im1_.rows;

	im2_.create(cv::Size(b.width(), b.height()), CV_8UC4);
	b.to_gpumat().download(im2_);
	cv::flip(im2_, im2_, 0);

	if (im2_.cols != im1_.cols || im2_.rows != im1_.rows) {
		throw FTL_Error("Left and right images are different sizes");
	}

	new_frame_.clear();
}

void ftl::gui::Camera::_downloadFrame(ftl::cuda::TextureObject<uchar4> &a) {
	im1_.create(cv::Size(a.width(), a.height()), CV_8UC4);
	a.to_gpumat().download(im1_);

	// OpenGL (0,0) bottom left
	cv::flip(im1_, im1_, 0);

	width_ = im1_.cols;
	height_ = im1_.rows;

	im2_ = cv::Mat();

	new_frame_.clear();
}

void ftl::gui::Camera::update(int fsid, const ftl::codecs::Channels<0> &c) {
	if (!isVirtual() && ((1 << fsid) & fsmask_)) {
		channels_ = c;
		if (c.has(Channel::Depth)) {
			//channels_ += Channel::ColourNormals;
		}
	}
}

void ftl::gui::Camera::update(std::vector<ftl::rgbd::FrameSet*> &fss) {
	UNIQUE_LOCK(mutex_, lk);

	framesets_ = &fss;

	//if (fss.size() <= fsid_) return;
	if (fid_ == 255) {
		name_ = "Virtual Camera";
		// Do a draw if not active. If active the draw function will be called
		// directly.
		if (screen_->activeCamera() != this) {
			_draw(fss);
		}
	} else {
		for (auto *fs : fss) {
			if (!usesFrameset(fs->id)) continue;

			ftl::rgbd::Frame *frame = nullptr;

			if ((size_t)fid_ >= fs->frames.size()) return;
			frame = &fs->frames[fid_];

			auto &buf = colouriser_->colourise(*frame, channel_, 0);
			if (isStereo() && frame->hasChannel(Channel::Right)) {
				_downloadFrames(buf, frame->createTexture<uchar4>(Channel::Right));
			} else {
				_downloadFrame(buf);
			}

			auto n = frame->get<std::string>("name");
			if (n) {
				name_ = *n;
			} else {
				name_ = "No name";
			}
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

bool ftl::gui::Camera::thumbnail(cv::Mat &thumb) {
	{
		UNIQUE_LOCK(mutex_, lk);
		if (im1_.empty()) return false;
		// FIXME: Use correct aspect ratio?
		cv::resize(im1_, thumb, cv::Size(320,180));
	}
	cv::flip(thumb, thumb, 0);
	return true;
}

void ftl::gui::Camera::active(bool a) {
	if (a) {

	} else {
		neye_[0] = eye_[0];
		neye_[1] = eye_[1];
		neye_[2] = eye_[2];
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
		if (screen_->isVR()) {
			#ifdef HAVE_OPENVR
			
			vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);
			vr::VRCompositor()->WaitGetPoses(rTrackedDevicePose_, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

			if (isStereo() && rTrackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
			{
				Eigen::Matrix4d eye_l = ConvertSteamVRMatrixToMatrix4(
					vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));
				
				Eigen::Matrix4d eye_r = ConvertSteamVRMatrixToMatrix4(
					vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));

				float baseline_in = 2.0 * eye_l(0, 3);
				
				if (baseline_in != baseline_) {
					baseline_ = baseline_in;
					//src_->set("baseline", baseline_);
					state_.getLeft().baseline = baseline_;
					state_.getRight().baseline = baseline_;
				}
				Eigen::Matrix4d pose = ConvertSteamVRMatrixToMatrix4(rTrackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
				Eigen::Vector3d ea = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

				//LOG(INFO) << "Pose: " << pose;
				eye_[0] = pose(0, 3);
				eye_[1] = -pose(1, 3);
				eye_[2] = -pose(2, 3);
				
				// NOTE: If modified, should be verified with VR headset!
				Eigen::Matrix3d R;
				R =		Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX()) *
						Eigen::AngleAxisd(-ea[1], Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(-ea[2], Eigen::Vector3d::UnitZ()); 
				
				//double rd = 180.0 / 3.141592;
				//LOG(INFO) << "rotation x: " << ea[0] *rd << ", y: " << ea[1] * rd << ", z: " << ea[2] * rd;
				// pose.block<3, 3>(0, 0) = R;
				
				rotmat_.block(0, 0, 3, 3) = R;
			
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
		}

		Eigen::Translation3d trans(eye_);
		Eigen::Affine3d t(trans);
		Eigen::Matrix4d viewPose = t.matrix() * rotmat_;

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

		{
			//UNIQUE_LOCK(mutex_, lk);
			if (im1_.rows != 0) {
				texture1_.update(im1_);
			}
			if (isStereo() && im2_.rows != 0) {
				texture2_.update(im2_);
			}
		}
	}

	//return texture1_;
}

void ftl::gui::Camera::snapshot(const std::string &filename) {
	cv::Mat flipped;

	{
		UNIQUE_LOCK(mutex_, lk);
		cv::flip(im1_, flipped, 0);
	}
	cv::cvtColor(flipped, flipped, cv::COLOR_BGRA2BGR);
	cv::imwrite(filename, flipped);
}

void ftl::gui::Camera::startVideoRecording(const std::string &filename) {
	if (!record_stream_) {
		record_stream_ = ftl::create<ftl::stream::File>(screen_->root(), "video2d");
		record_stream_->setMode(ftl::stream::File::Mode::Write);
		record_sender_ = ftl::create<ftl::stream::Sender>(screen_->root(), "videoEncode");
		record_sender_->setStream(record_stream_);
	}

	if (record_stream_->active()) return;

	record_stream_->set("filename", filename);
	record_stream_->begin();
}

void ftl::gui::Camera::stopVideoRecording() {
	if (record_stream_ && record_stream_->active()) record_stream_->end();
}

