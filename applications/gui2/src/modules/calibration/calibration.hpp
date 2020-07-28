#pragma once

#include "../../module.hpp"

#include <ftl/calibration/object.hpp>
#include <ftl/calibration/extrinsic.hpp>
#include <ftl/calibration/structures.hpp>
#include <opencv2/core/types.hpp>

namespace ftl
{
namespace gui2
{

/** OpenCV calibration flags */
class OpenCVCalibrateFlags {
public:
	bool has(unsigned int flag) const { return (flags_ & flag) != 0; }
	void set(unsigned int flag) { flags_ |= flag; }
	void unset(unsigned int  flag) { flags_ &= ~flag; }
	void reset() { flags_ = 0; }
	std::string name(int) const;
	operator int() { return flags_; }

	virtual int defaultFlags() const;
	virtual std::vector<int> list() const;
	virtual std::string explain(int) const;

private:
	int flags_ = 0;
};

class  OpenCVCalibrateFlagsStereo : public OpenCVCalibrateFlags {
public:
	int defaultFlags() const override;
	std::vector<int> list() const override;
	std::string explain(int) const override;
};

/**
 * Calibration. Loads Intrinsic and Extrinsic calibration modules and
 * adds buttons to main screen.
 */
class Calibration : public Module {
public:
	using Module::Module;
	virtual ~Calibration();

	virtual void init() override;
};

/**
 * Calibration base module. Implements methods to loading/saving calibration.
 * Also manages enabling/disabling calibration.
 */

class CalibrationModule : public Module {
public:
	using Module::Module;
	virtual void init() = 0;

protected:
	/** Set new calibration. */
	void setCalibration(ftl::data::Frame& frame, ftl::calibration::CalibrationData data);

	/** Activate/deactivate calibration (rectification, distortion corrections,
	 *  ...). See also StereoVideo */
	/** set mode, update performed by checkFrame() when next called */
	void setCalibrationMode(bool value);
	/** set mode directly to frame */
	void setCalibrationMode(ftl::data::Frame& frame, bool value);

	/** Check everything is in expected state. If returns true, processing can
	 * continue. Use this in frameset callback. Also sets calibration mode if
	 * it doesn't match with stored state. Should always be called in FrameSet
	 * callback.
	 */
	bool checkFrame(ftl::data::Frame& frame);

	cv::cuda::GpuMat getGpuMat(ftl::rgbd::Frame&, ftl::codecs::Channel);
	cv::Mat getMat(ftl::rgbd::Frame&, ftl::codecs::Channel);

private:
	bool calibrationEnabled(ftl::data::Frame& frame);

	std::atomic_bool wait_update_ = false;
	std::atomic_bool calibration_enabled_ = false;
	ftl::Handle update_handle_;
};

/**
 * GUI for camera intrinsic calibration. Only sources which have CalibrationData
 * channel can be calibrated (StereoVideo receives updates and saves them).
 *
 * TODO: Catch exceptions in future and report back to GUI. At the moment
 *		 errors are printed with logging.
 * TODO: View: add button to get back to chessboard/capture parameters.
 * TODO: Saving calibration should give more feedback, saved just tells it was
 * 		 sent but it does not verify it was received (or know if it was
 * 		 successfully saved; if saving fails do not write file/changes; how
 * 		 to inform GUI/client about the error?)
 *
 * TODO: FEATURE: Add timer to calibration window showing remaining time until
 * 		 next picture is captured.
 * TODO: FEATURE: Add calibration image window for browsing calibration images
 * 		 and discarding bad images manually. Also should support visualization
 * 		 of calibration results; draw detected points and re-projected points
 * 		 using OpenGL (reproject points implemented in calibration:: using
 * 		 with OpenCV).
 * TODO: FEATURE: Visualize lens distortion. Plot regular grid and apply
 * 		 distortion model.
 */
class IntrinsicCalibration : public CalibrationModule {
public:
	using CalibrationModule::CalibrationModule;

	virtual void init() override;
	virtual ~IntrinsicCalibration();

	/** start calibration process, replaces active view */
	void start(ftl::data::FrameID id);

	bool hasChannel(ftl::codecs::Channel c);
	/** select channel */
	void setChannel(ftl::codecs::Channel c);
	ftl::codecs::Channel channel() { return state_->channel; }

	int count() { return state_->count; }
	int calibrated() { return state_->calibrated; }

	OpenCVCalibrateFlags& flags() { return state_->flags; };
	int defaultFlags();

	/** Reset calibration instance, discards drops all state. */
	void reset();

	void setChessboard(cv::Size, double);
	cv::Size chessboardSize();
	double squareSize();

	/** Returns if capture/calibration is still processing in background.
	 * calib() instance must not be modifed while isBusy() is true.
	 */
	bool isBusy();

	/** Start/stop capture. After stopping, use isBusy() to check when last
	 * frame is finished.
	 */
	void setCapture(bool v) { state_->capture = v; }
	bool capturing() { return state_->capture; }

	/** get/set capture frequency: interval between processed frames in
	 * chessboard detection
	*/
	void setFrequency(float v) { state_->frequency = v; }
	float frequency() { return state_->frequency; }

	int maxIter() { return state_->max_iter; }
	void setMaxIter(int v) { state_->max_iter = v; }

	/** Run calibration in another thread. Check status with isBusy(). */
	void run();

	/** Save calibration */
	void save();

	ftl::calibration::CalibrationData::Intrinsic calibration();

	float reprojectionError() { return state_->reprojection_error; }

	/** Get sensor size from config/previous calibration (in mm) */
	cv::Size2d sensorSize();
	void setSensorSize(cv::Size2d size);

	/** Set/get focal length in mm */
	double focalLength();
	void setFocalLength(double value, cv::Size2d sensor_size);

	/** Set principal point at image center */
	void resetPrincipalPoint();

	void resetDistortion();

	/** get current frame */
	cv::cuda::GpuMat getFrame();
	bool hasFrame();

	cv::cuda::GpuMat getFrameUndistort();

	/** get previous points (visualization) */
	std::vector<cv::Point2f> previousPoints();
	// must not be running_
	//std::vector<cv::Point2f> getPoints(int n);
	//std::vector<cv::Point2f> getProjectedPoints(int n);

	/** List sources which can be calibrated.
	 */
	std::vector<std::pair<std::string, ftl::data::FrameID>> listSources(bool all=false);

private:
	bool onFrame_(const ftl::data::FrameSetPtr& fs);
	/** Set actual channel (channel_alt_) to high res if found in fs */
	void setChannel_(ftl::data::FrameSetPtr fs);

	std::future<void> future_;
	std::mutex mtx_;
	ftl::data::FrameSetPtr fs_current_;
	ftl::data::FrameSetPtr fs_update_;

	struct State {
		cv::Mat gray;

		ftl::codecs::Channel channel;
		ftl::codecs::Channel channel_alt;
		ftl::data::FrameID id;

		std::atomic_bool capture = false;
		std::atomic_bool running = false;
		float last = 0.0f;
		float frequency = 0.5f;
		bool calibrated = false;
		int count = 0;
		int max_iter = 50;
		float reprojection_error = NAN;
		std::vector<std::vector<cv::Point2f>> points;
		std::vector<std::vector<cv::Point3f>> points_object;

		std::unique_ptr<ftl::calibration::ChessboardObject> object;
		OpenCVCalibrateFlags flags;
		ftl::calibration::CalibrationData::Intrinsic calib;
	};

	std::unique_ptr<State> state_;
};

////////////////////////////////////////////////////////////////////////////////

/**
 * GUI for camera extrinsic calibration. Sources must be in same FrameSet
 * (synchronization) and have CalibrationData channel. Provided extrinsic
 * parameters can be used to calculate paramters for stereo rectification.
 */

class ExtrinsicCalibration : public CalibrationModule {
public:
	using CalibrationModule::CalibrationModule;

	virtual void init() override;
	virtual ~ExtrinsicCalibration();

	/** List framesets and calibrateable sources */
	std::vector<std::pair<std::string, unsigned int>> listFrameSets();
	std::vector<std::pair<std::string, ftl::data::FrameID>> listSources(unsigned int fsid, bool all);

	/** start calibration process for given frames. Assumes stereo,
	 * calibration: left and right channels are used. */
	void start(unsigned int fsid, std::vector<ftl::data::FrameID> sources);

	/** discard current state and load defaults */
	void reset();

	int cameraCount();

	ftl::calibration::ExtrinsicCalibration& calib();

	/** hasFrame(int) must be true before calling getFrame() **/
	bool hasFrame(int camera);
	const cv::cuda::GpuMat getFrame(int camera);
	const cv::cuda::GpuMat getFrameRectified(int camera);

	/** Next FrameSet, returns true if new FrameSet is available */
	bool next();

	bool capturing();
	void setCapture(bool value);

	/** Set callback for point detection. Callback returns number of points
	 * found, takes input frame, channel and output points as arguments.
	 */
	//void setCallback(const std::function<int(cv::InputArray, const cv::Mat&, const cv::Mat&, std::vector<cv::Point2f>&)>& cb) { cb_detect_ = cb; }

	struct CameraID : ftl::data::FrameID {
		CameraID(unsigned int fs, unsigned int s, ftl::codecs::Channel channel) :
			ftl::data::FrameID::FrameID(fs, s), channel(channel) {}
		const ftl::codecs::Channel channel;
	};

	/** list selected (active) cameras */
	std::vector<CameraID> cameras();

	/** Run calibration in another thread. Check status with isBusy(). */
	void run();

	/** Returns if capture/calibration is still processing in background.
	 * calib() instance must not be modifed while isBusy() is true.
	 */
	bool isBusy();

	/** status message */
	std::string status() { return state_.calib.status(); }

	/** Get previous points (for visualization) */
	const std::vector<cv::Point2d>& previousPoints(int camera);

	/** Get number of frames captured by a camera */
	int getFrameCount(int c);

	void updateCalibration(int c);
	void updateCalibration();

	void saveInput(const std::string& filename);
	void loadInput(const std::string& filename);

	ftl::calibration::CalibrationData::Calibration calibration(int camera);

	double reprojectionError(int camera=-1);

	enum Flags {
		ZERO_DISTORTION = 1,
		RATIONAL_MODEL = 2,
		FIX_INTRINSIC = 4,
		FIX_FOCAL = 8,
		FIX_PRINCIPAL_POINT = 16,
		FIX_DISTORTION = 32,
		LOSS_CAUCHY = 64,
		NONMONOTONIC_STEP = 128,
	};

	void setFlags(int flags);
	int flags() const;

protected:
	ftl::calibration::CalibrationData::Calibration getCalibration(CameraID id);

	/** Calculate stereo rectification maps for two cameras; state_.maps[1,2]
	 * must already be initialized at correct size */
	void stereoRectify(int cl, int cr,
		const ftl::calibration::CalibrationData::Calibration& l,
		const ftl::calibration::CalibrationData::Calibration& r);

private:
	// map frameid+channel to int. used by ExtrinsicCalibration
	struct Camera {
		const CameraID id;
		ftl::calibration::CalibrationData::Calibration calib;
	};

	bool onFrameSet_(const ftl::data::FrameSetPtr& fs);

	std::future<void> future_;
	std::atomic_bool running_;
	ftl::data::FrameSetPtr fs_current_;
	ftl::data::FrameSetPtr fs_update_;

	struct State {
		bool capture = false;
		int min_cameras = 2;
		int flags = 0;
		std::vector<Camera> cameras;

		std::unique_ptr<ftl::calibration::CalibrationObject> calib_object;
		ftl::calibration::ExtrinsicCalibration calib;
		std::vector<std::vector<cv::Point2d>> points_prev;
		std::vector<cv::cuda::GpuMat> maps1;
		std::vector<cv::cuda::GpuMat> maps2;
	};
	State state_;
};

////////////////////////////////////////////////////////////////////////////////

/** Stereo calibration for OpenCV's calibrateStereo() */

class StereoCalibration : public CalibrationModule {
public:
	using CalibrationModule::CalibrationModule;
	virtual void init() override;
	virtual ~StereoCalibration();

	/** start calibration process, replaces active view */
	void start(ftl::data::FrameID id);

	bool hasChannel(ftl::codecs::Channel c);

	void setChessboard(cv::Size, double);
	cv::Size chessboardSize();
	double squareSize();

	/** Reset calibration instance, discards drops all state. */
	void reset();

	OpenCVCalibrateFlagsStereo& flags() { return state_->flags; };
	void resetFlags();

	/** Returns if capture/calibration is still processing in background.
	 * calib() instance must not be modifed while isBusy() is true.
	 */
	bool isBusy();

	/** Start/stop capture. After stopping, use isBusy() to check when last
	 * frame is finished.
	 */
	void setCapture(bool v);
	bool capturing();

	/** get/set capture frequency: interval between processed frames in
	 * chessboard detection
	*/
	void setFrequency(float v);
	float frequency();

	/** Run calibration in another thread. Check status with isBusy(). */
	void run();

	/** Save calibration */
	void save();

	/** check if calibration valid: baseline > 0 */
	bool calibrated();

	/** get current frame */
	cv::cuda::GpuMat getLeft();
	cv::cuda::GpuMat getRight();
	cv::cuda::GpuMat getLeftRectify();
	cv::cuda::GpuMat getRightRectify();
	bool hasFrame();

	ftl::calibration::CalibrationData::Calibration calibrationLeft();
	ftl::calibration::CalibrationData::Calibration calibrationRight();
	double baseline();

	/** get previous points (visualization) */
	std::vector<std::vector<cv::Point2f>> previousPoints();
	cv::cuda::GpuMat getLeftPrevious();
	cv::cuda::GpuMat getRightPrevious();
	int count() const { return state_->count; }
	/** List sources which can be calibrated.
	 */
	std::vector<std::pair<std::string, ftl::data::FrameID>> listSources(bool all=false);

private:
	bool onFrame_(const ftl::data::FrameSetPtr& fs);
	void calculateRectification();
	ftl::rgbd::Frame& frame_();

	ftl::codecs::Channel channelLeft_();
	ftl::codecs::Channel channelRight_();

	std::future<void> future_;
	std::mutex mtx_;
	ftl::data::FrameSetPtr fs_current_;
	ftl::data::FrameSetPtr fs_update_;

	struct State {
		cv::Mat gray_left;
		cv::Mat gray_right;

		ftl::calibration::CalibrationData calib;
		std::unique_ptr<ftl::calibration::ChessboardObject> object;
		ftl::data::FrameID id;
		bool highres = false;
		cv::Size imsize;
		std::atomic_bool capture = false;
		std::atomic_bool running = false;
		float last = 0.0f;
		float frequency = 0.5f;
		int count = 0;
		float reprojection_error = NAN;
		OpenCVCalibrateFlagsStereo flags;

		// maps for rectification (cv)
		std::pair<cv::Mat, cv::Mat> map_l;
		std::pair<cv::Mat, cv::Mat> map_r;
		cv::Rect validROI_l;
		cv::Rect validROI_r;

		ftl::data::FrameSetPtr fs_previous_points;
		std::vector<std::vector<cv::Point2f>> points_l;
		std::vector<std::vector<cv::Point2f>> points_r;
		std::vector<std::vector<cv::Point3f>> points_object;
	};
	std::unique_ptr<State> state_;
};

}
}
