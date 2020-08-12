#include <ftl/streams/renderer.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/cuda/transform.hpp>
#include <ftl/operators/antialiasing.hpp>
#include <ftl/operators/gt_analysis.hpp>
#include <ftl/operators/poser.hpp>
#include <ftl/codecs/shapes.hpp>

#include <loguru.hpp>

#include "openvr_render.hpp"
#include "collisions.hpp"

#include <GL/gl.h>

using ftl::render::Source;
using ftl::render::OpenVRRender;
using ftl::codecs::Channel;
using ftl::rgbd::Capability;
using ftl::codecs::Shape3DType;

#ifdef HAVE_OPENVR
static vr::IVRSystem *HMD = nullptr;
#endif


OpenVRRender::OpenVRRender(ftl::render::Source *host, ftl::stream::Feed *feed)
: ftl::render::BaseSourceImpl(host), feed_(feed), my_id_(0), post_pipe_(nullptr), baseline_(0.06f) {
	#ifdef HAVE_OPENVR
	if (HMD) throw FTL_Error("Can only have one OpenVR device");
	#endif
	initVR();

	renderer_ = std::unique_ptr<ftl::render::CUDARender>(
		ftl::create<ftl::render::CUDARender>(host_, "renderer")
	);

	renderer2_ = std::unique_ptr<ftl::render::CUDARender>(
		ftl::create<ftl::render::CUDARender>(host_, "renderer2")
	);

	intrinsics_ = ftl::create<ftl::Configurable>(host_, "intrinsics");

	filter_ = nullptr;
	std::string source = host_->value("source", std::string(""));

	if (source.size() > 0) {
		filter_ = feed_->filter({source},{Channel::Colour, Channel::Depth});
	} else {
		filter_ = feed_->filter({Channel::Colour, Channel::Depth});
	}

	host_->on("source", [this]() {
		std::string source = host_->value("source", std::string(""));

		if (source.size() > 0) {
			if (filter_) filter_->remove();
			filter_ = feed_->filter({source},{Channel::Colour, Channel::Depth});
		} else {
			if (filter_) filter_->remove();
			filter_ = feed_->filter({Channel::Colour, Channel::Depth});
		}
	});

	eye_ = Eigen::Vector3d::Zero();
	rotmat_.setIdentity();
	initial_pose_.setIdentity();

	host_->value("reset_pose", false);
	host_->on("reset_pose", [this]() {
		pose_calibrated_.clear();
	});
}

OpenVRRender::~OpenVRRender() {
	if (filter_) filter_->remove();
	delete intrinsics_;
	if (post_pipe_) delete post_pipe_;

	#ifdef HAVE_OPENVR
	if (HMD != nullptr) {
		vr::VR_Shutdown();
	}
	#endif
}

bool OpenVRRender::initVR() {
	#ifdef HAVE_OPENVR
	if (!vr::VR_IsHmdPresent()) {
		return false;
	}

	if (HMD) return true;

	vr::EVRInitError eError = vr::VRInitError_None;
	HMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
	
	if (eError != vr::VRInitError_None)
	{
		HMD = nullptr;
		LOG(ERROR) << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError);
		return false;
	}

	return true;
	#else
	return false;
	#endif
}

bool OpenVRRender::supported() {
	#ifdef HAVE_OPENVR
	return vr::VR_IsHmdPresent();
	#else
	return false;
	#endif
}

bool OpenVRRender::isReady() {
	#ifdef HAVE_OPENVR
	return HMD != nullptr;
	#else
	return false;
	#endif
}

bool OpenVRRender::capture(int64_t ts) {
	return true;
}

#ifdef HAVE_OPENVR
static inline Eigen::Matrix4d ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose )
{
	Eigen::Matrix4d matrixObj;
	matrixObj <<
		matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
		matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
		matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
					0.0,			 0.0,			  0.0,			   1.0;
	return matrixObj;
}

static inline Eigen::Matrix4d ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix44_t &matPose )
{
	Eigen::Matrix4d matrixObj;
	matrixObj <<
		matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
		matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
		matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
		matPose.m[3][0], matPose.m[3][1], matPose.m[3][2], matPose.m[3][3];
	return matrixObj;
}

static Eigen::Matrix3d getCameraMatrix(const double tanx1,
								const double tanx2,
								const double tany1,
								const double tany2,
								const double size_x,
								const double size_y) {
	
	Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
	
	CHECK(tanx1 < 0 && tanx2 > 0 && tany1 < 0 && tany2 > 0);
	CHECK(size_x > 0 && size_y > 0);

	double fx = size_x / (-tanx1 + tanx2);
	double fy = size_y / (-tany1 + tany2);
	C(0,0) = fx;
	C(1,1) = fy;
	C(0,2) = tanx1 * fx;
	C(1,2) = tany1 * fy;

	// safe to remove
	CHECK((int) (abs(tanx1 * fx) + abs(tanx2 * fx)) == (int) size_x);
	CHECK((int) (abs(tany1 * fy) + abs(tany2 * fy)) == (int) size_y);

	return C;
}

static Eigen::Matrix3d getCameraMatrix(vr::IVRSystem *vr, const vr::Hmd_Eye &eye) {
	float tanx1, tanx2, tany1, tany2;
	uint32_t size_x, size_y;
	vr->GetProjectionRaw(eye, &tanx1, &tanx2, &tany1, &tany2);
	vr->GetRecommendedRenderTargetSize(&size_x, &size_y);
	return getCameraMatrix(tanx1, tanx2, tany1, tany2, size_x, size_y);
}
#endif

bool OpenVRRender::retrieve(ftl::data::Frame &frame_out) {

	#ifdef HAVE_OPENVR
	//auto input = std::atomic_load(&input_);

	my_id_ = frame_out.frameset();

	auto sets = filter_->getLatestFrameSets();

	if (sets.size() > 0) {
		ftl::rgbd::Frame &rgbdframe = frame_out.cast<ftl::rgbd::Frame>();

		if (!frame_out.has(Channel::Calibration)) {
			auto &left = rgbdframe.setLeft();
			auto &right = rgbdframe.setRight();

			left = ftl::rgbd::Camera::from(intrinsics_);
			right = ftl::rgbd::Camera::from(intrinsics_);

			left.baseline = baseline_;
			right.baseline = baseline_;
			
			unsigned int size_x, size_y;
			HMD->GetRecommendedRenderTargetSize(&size_x, &size_y);
			left.width = size_x;
			left.height = size_y;
			right.width = size_x;
			right.height = size_y;

			Eigen::Matrix3d intrinsic;

			intrinsic = getCameraMatrix(HMD, vr::Eye_Left);
			CHECK(intrinsic(0, 2) < 0 && intrinsic(1, 2) < 0);
			left.fx = intrinsic(0,0);
			left.fy = intrinsic(0,0);
			left.cx = intrinsic(0,2);
			left.cy = intrinsic(1,2);

			intrinsic = getCameraMatrix(HMD, vr::Eye_Right);
			CHECK(intrinsic(0, 2) < 0 && intrinsic(1, 2) < 0);
			right.fx = intrinsic(0,0);
			right.fy = intrinsic(0,0);
			right.cx = intrinsic(0,2);
			right.cy = intrinsic(1,2);

			LOG(INFO) << "VR Left Intrinsics: fx=" << left.fx << ",cx=" << left.cx << ",cy=" << left.cy;
			LOG(INFO) << "VR Right Intrinsics: fx=" << right.fx << ",cx=" << right.cx << ",cy=" << right.cy;

			if (!frame_out.has(Channel::Capabilities)) {
				auto &cap = frame_out.create<std::unordered_set<Capability>>(Channel::Capabilities);
				cap.emplace(Capability::VIDEO);
				cap.emplace(Capability::MOVABLE);
				cap.emplace(Capability::ADJUSTABLE);
				cap.emplace(Capability::VIRTUAL);
				cap.emplace(Capability::LIVE);
				cap.emplace(Capability::VR);
			}

			auto &meta = frame_out.create<std::map<std::string,std::string>>(Channel::MetaData);
			meta["name"] = host_->value("name", host_->getID());
			meta["id"] = host_->getID();
			meta["uri"] = std::string("device:openvr");
			meta["device"] = std::string("OpenVR Render");
		}
		//if (!frame_out.has(Channel::Pose)) {
		//	rgbdframe.setPose() = Eigen::Matrix4d::Identity();
		//}

		int width = rgbdframe.getLeft().width;
		int height = rgbdframe.getLeft().height;

		vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);
		auto vrerr = vr::VRCompositor()->WaitGetPoses(rTrackedDevicePose_, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

		if (vrerr != vr::VRCompositorError_None) {
			frame_out.message(ftl::data::Message::Error_OPENVR, "Could not get VR pose");
			LOG(ERROR) << "Error getting VR poses: " << (int)vrerr;
		}

		if (rTrackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid ) {
			Eigen::Matrix4d eye_l = ConvertSteamVRMatrixToMatrix4(
				vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));

			//Eigen::Matrix4d eye_r = ConvertSteamVRMatrixToMatrix4(
			//	vr::VRSystem()->GetEyeToHeadTransform(vr::Eye_Left));

			float baseline_in = 2.0 * eye_l(0, 3);

			if (baseline_in != baseline_) {
				baseline_ = baseline_in;
				auto cur_left = rgbdframe.getLeft();
				cur_left.baseline = baseline_;
				rgbdframe.setLeft() = cur_left;

				auto cur_right = rgbdframe.getRight();
				cur_right.baseline = baseline_;
				rgbdframe.setRight() = cur_right;

				LOG(INFO) << "VR Baseline: " << baseline_;
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

			// TODO: Somehow allow adjustment in addition to the VR pose...
			//eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
			//eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
			//eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

			Eigen::Translation3d trans(eye_ + vreye);
			Eigen::Affine3d t(trans);
			auto viewPose = t.matrix() * rotmat_;

			if (!pose_calibrated_.test_and_set()) {
				if (pose_calibration_start_ == -1) pose_calibration_start_ = ftl::timer::get_time();

				std::string headset_origin = host_->value("headset_origin", std::string(""));
				Eigen::Matrix4d horigin;
				horigin.setIdentity();

				if (headset_origin.size() > 0) {
					ftl::operators::Poser::get(headset_origin, horigin);
				}
				initial_pose_ = horigin*viewPose.inverse();

				if (host_->value("reset_pose", false) && ftl::timer::get_time() < pose_calibration_start_ + host_->value("calibration_time",10000)) {
					pose_calibrated_.clear();
				} else {
					pose_calibration_start_ = -1;
				}
			}

			rgbdframe.setPose() = initial_pose_*viewPose;

		} else {
			LOG(ERROR) << "No VR Pose";
			frame_out.message(ftl::data::Message::Error_OPENVR, "Could not get VR pose");
			rgbdframe.setPose().setIdentity();
		}

		// TODO: Get controller data if available...

		texture1_.make(width, height, ftl::utility::GLTexture::Type::BGRA);
		texture2_.make(width, height, ftl::utility::GLTexture::Type::BGRA);
		
		rgbdframe.create<cv::cuda::GpuMat>(Channel::Colour) = texture1_.map(renderer_->getCUDAStream());
		rgbdframe.create<cv::cuda::GpuMat>(Channel::Colour2) = texture2_.map(renderer2_->getCUDAStream());
		rgbdframe.create<cv::cuda::GpuMat>(Channel::Depth).create(height, width, CV_32F);
		rgbdframe.createTexture<float>(Channel::Depth);

		rgbdframe.set<ftl::rgbd::VideoFrame>(Channel::Colour).setOpenGL(texture1_.texture());
		rgbdframe.set<ftl::rgbd::VideoFrame>(Channel::Colour2).setOpenGL(texture2_.texture());

		auto shapes = rgbdframe.create<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

		/*Eigen::Matrix4d origin;
		origin.setIdentity();
		std::string origin_name = host_->value("origin", std::string(""));
		if (origin_name.size() > 0) {
			ftl::operators::Poser::get(origin_name, origin);
		}*/

		try {
			renderer_->begin(rgbdframe, ftl::codecs::Channel::Left);
			renderer2_->begin(rgbdframe, Channel::Colour2);

			for (auto &s : sets) {
				if (s->frameset() == my_id_) continue;  // Skip self

				Eigen::Matrix4d pose;
				pose.setIdentity();
				if (s->hasChannel(Channel::Pose)) pose = s->cast<ftl::rgbd::Frame>().getPose();

				// TODO: Check frame has required channels?

				// FIXME: Don't use identity transform, get from Poser somehow.
				renderer_->submit(
					s.get(),
					ftl::codecs::Channels<0>(ftl::codecs::Channel::Colour),
					pose);

				renderer2_->submit(
					s.get(),
					ftl::codecs::Channels<0>(ftl::codecs::Channel::Colour),
					pose);
			}

			renderer_->render();
			renderer2_->render();

			// Now do CPU-based render jobs
			for (auto &s : sets) {
				if (s->frameset() == my_id_) continue;  // Skip self

				// Inject and copy data items and mix audio
				for (size_t i=0; i<s->frames.size(); ++i) {
					auto &f = s->frames[i];

					// If audio is present, mix with the other frames
					if (f.hasChannel(Channel::AudioStereo)) {
						// Map a mixer track to this frame
						auto &mixmap = mixmap_[f.id().id];
						if (mixmap.track == -1) {
							mixmap.track = mixer_.add(f.name());
						}

						// Do mix but must not mix same frame multiple times
						if (mixmap.last_timestamp != f.timestamp()) {
							const auto &audio = f.get<std::list<ftl::audio::Audio>>(Channel::AudioStereo).front();
							mixer_.write(mixmap.track, audio.data());
							mixmap.last_timestamp = f.timestamp();
						}
					}

					// Add pose as a camera shape
					auto &shape = shapes.list.emplace_back();
					shape.id = f.id().id;
					shape.type = Shape3DType::CAMERA;
					shape.size = Eigen::Vector3f(0.2f,0.2f,0.2f);
					shape.pose = f.cast<ftl::rgbd::Frame>().getPose().cast<float>();
					shape.label = f.name();

					// Copy all original shapes
					if (f.hasChannel(Channel::Shapes3D)) {
						const auto &fshapes = f.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);
						shapes.list.insert(shapes.list.end(), fshapes.begin(), fshapes.end());
					}
				}
			}

			mixer_.mix();

			// Write mixed audio to frame.
			if (mixer_.frames() > 0) {
				auto &list = frame_out.create<std::list<ftl::audio::Audio>>(Channel::AudioStereo).list;
				list.clear();

				int fcount = mixer_.frames();
				mixer_.read(list.emplace_front().data(), fcount);
			}

			// TODO: Blend option

			renderer_->end();
			renderer2_->end();
		} catch (const ftl::exception &e) {
			LOG(ERROR) << "Render exception: " << e.what();
			renderer_->cancel();
			renderer2_->cancel();
			frame_out.message(ftl::data::Message::Error_RENDER, e.what());
		}

		if (!post_pipe_) {
			post_pipe_ = ftl::config::create<ftl::operators::Graph>(host(), "post_filters");
			post_pipe_->append<ftl::operators::Poser>("poser");
			post_pipe_->append<ftl::operators::FXAA>("fxaa");
			post_pipe_->append<ftl::operators::GTAnalysis>("gtanalyse");
		}

		post_pipe_->apply(rgbdframe, rgbdframe, 0);

		if (host_->value("enable_touch", false)) {
			ftl::render::collision2touch(rgbdframe, renderer_->getCollisions(), sets, my_id_, host_->value("touch_min", 0.01f), host_->value("touch_max", 0.05f));
		}

		// FIXME: Use a stream
		ftl::cuda::flip<uchar4>(rgbdframe.set<cv::cuda::GpuMat>(Channel::Colour), 0);
		ftl::cuda::flip<uchar4>(rgbdframe.set<cv::cuda::GpuMat>(Channel::Colour2), 0);

		texture1_.unmap(renderer_->getCUDAStream());
		texture2_.unmap(renderer2_->getCUDAStream());
		//return true;


		// Send left and right textures to VR headset
		vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)texture1_.texture(), vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
		vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)texture2_.texture(), vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

		glFlush();

	}

	return true;

	#else
	return false;
	#endif
}
