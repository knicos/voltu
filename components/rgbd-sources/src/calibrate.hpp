/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <opencv2/opencv.hpp>
#include "local.hpp"
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace cv {
class FileStorage;
class FileNode;
};

namespace ftl {

/**
 * Manage both performing and applying camera calibration. It will attempt to
 * load any existing cached camera calibration unless explicitely told to
 * redo the calibration.
 */
class Calibrate : public ftl::Configurable {
	public:
	
	// TODO(nick) replace or remove this class.
	class Settings {
		public:
		Settings() : calibrationPattern(CHESSBOARD), squareSize(50.0f),
			nrFrames(30), aspectRatio(1.0f), delay(100), writePoints(false), writeExtrinsics(true),
			writeGrid(false), calibZeroTangentDist(false), calibFixPrincipalPoint(true),
			flipVertical(false), showUndistorsed(true), useFisheye(false), fixK1(false),
			fixK2(false), fixK3(false), fixK4(false), fixK5(false), cameraID(0), atImageList(0),
			inputType(INVALID), goodInput(false), flag(0) {}
		enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
		enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

		void write(cv::FileStorage& fs) const;
		void read(ftl::Configurable *node);
		void validate();
		//Mat nextImage();

		static bool readStringList( const std::string& filename, std::vector<std::string>& l );

		static bool isListOfImages( const std::string& filename);
		public:
		cv::Size boardSize;              // The size of the board -> Number of items by width and height
		Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
		float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
		int nrFrames;                // The number of frames to use from the input for calibration
		float aspectRatio;           // The aspect ratio
		float delay;                   // In case of a video input
		bool writePoints;            // Write detected feature points
		bool writeExtrinsics;        // Write extrinsic parameters
		bool writeGrid;              // Write refined 3D target grid points
		bool calibZeroTangentDist;   // Assume zero tangential distortion
		bool calibFixPrincipalPoint; // Fix the principal point at the center
		bool flipVertical;           // Flip the captured images around the horizontal axis
		std::string outputFileName;       // The name of the file where to write
		bool showUndistorsed;        // Show undistorted images after calibration
		std::string input;                // The input ->
		bool useFisheye;             // use fisheye camera model for calibration
		bool fixK1;                  // fix K1 distortion coefficient
		bool fixK2;                  // fix K2 distortion coefficient
		bool fixK3;                  // fix K3 distortion coefficient
		bool fixK4;                  // fix K4 distortion coefficient
		bool fixK5;                  // fix K5 distortion coefficient

		int cameraID;
		std::vector<std::string> imageList;
		size_t atImageList;
		//cv::VideoCapture inputCapture;
		InputType inputType;
		bool goodInput;
		int flag;

		private:
		std::string patternToUse;


	};
	public:
	Calibrate(nlohmann::json &config, ftl::LocalSource *s);
	
	/**
	 * Perform a new camera calibration. Ignore and replace any existing
	 * cached calibration.
	 */
	bool recalibrate();
	
	bool undistort(cv::Mat &l, cv::Mat &r);

	void distort(cv::Mat &rgb, cv::Mat &depth);
	
	/**
	 * Get both left and right images from local source, but with intrinsic
	 * and extrinsic stereo calibration already applied.
	 */
	bool rectified(cv::Mat &l, cv::Mat &r);

	/**
	 * Rectify and remove distortions from from images l and r using cv::remap()
	 */
	void rectifyStereo(cv::Mat &l, cv::Mat &r);

	bool isCalibrated();

	/**
	 * Get the camera matrix. Used to convert disparity map back to depth and
	 * a 3D point cloud.
	 */
	const cv::Mat &getQ() const { return Q_; }
	const cv::Mat &getCameraMatrix() const { return r1_; }

	private:
	bool _recalibrate(std::vector<std::vector<cv::Point2f>> *imagePoints,
		cv::Mat *cameraMatrix, cv::Mat *distCoeffs, cv::Size *imageSize);
	cv::Mat _nextImage(size_t cam);
	bool _loadCalibration();
	
	private:
	ftl::LocalSource *local_;
	Settings settings_;
	bool calibrated_;
	std::vector<cv::Mat> map1_;
	std::vector<cv::Mat> map2_;
	cv::Mat imap1_;
	cv::Mat imap2_;
	cv::Mat r1_;
	cv::Mat Q_;
};
};

#endif // _FTL_CALIBRATION_HPP_

