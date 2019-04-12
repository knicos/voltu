/*
 * Copyright 2019 Nicolas Pope
 */

#include <glog/logging.h>
#include <ftl/config.h>

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <ftl/calibrate.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using ftl::Calibrate;
using cv::FileStorage;
using cv::CALIB_FIX_PRINCIPAL_POINT;
using cv::CALIB_ZERO_TANGENT_DIST;
using cv::CALIB_FIX_ASPECT_RATIO;
using cv::CALIB_FIX_K1;
using cv::CALIB_FIX_K2;
using cv::CALIB_FIX_K3;
using cv::CALIB_FIX_K4;
using cv::CALIB_FIX_K5;
using cv::CALIB_ZERO_DISPARITY;
using cv::CALIB_USE_INTRINSIC_GUESS;
using cv::CALIB_SAME_FOCAL_LENGTH;
using cv::CALIB_RATIONAL_MODEL;
using cv::CALIB_CB_ADAPTIVE_THRESH;
using cv::CALIB_CB_NORMALIZE_IMAGE;
using cv::CALIB_CB_FAST_CHECK;
using cv::CALIB_USE_LU;
using cv::NORM_L2;
using cv::INTER_LINEAR;
using cv::COLOR_BGR2GRAY;
using cv::FileNode;
using cv::FileNodeIterator;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;
using cv::Matx33d;
using cv::Scalar;
using cv::Rect;
using cv::TermCriteria;
using cv::waitKey;
using std::string;
using std::vector;

void Calibrate::Settings::write(FileStorage& fs) const {
    fs << "{"
		<< "BoardSize_Width"  << boardSize.width
		<< "BoardSize_Height" << boardSize.height
		<< "Square_Size"         << squareSize
		<< "Calibrate_Pattern" << patternToUse
		<< "Calibrate_NrOfFrameToUse" << nrFrames
		<< "Calibrate_FixAspectRatio" << aspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint
		<< "Write_DetectedFeaturePoints" << writePoints
		<< "Write_extrinsicParameters"   << writeExtrinsics
		<< "Write_gridPoints" << writeGrid
		<< "Input_FlipAroundHorizontalAxis" << flipVertical
		<< "Input_Delay" << delay
		<< "}";
}

void Calibrate::Settings::read(const nlohmann::json& node) {
    boardSize.width = node["board_size"][0];
    boardSize.height = node["board_size"][1];
    squareSize = node["square_size"];
    nrFrames = node["num_frames"];
    aspectRatio = node["fix_aspect_ratio"];
    calibZeroTangentDist = node["assume_zero_tangential_distortion"];
    calibFixPrincipalPoint = node["fix_principal_point_at_center"];
    useFisheye =  node["use_fisheye_model"];
    flipVertical = node["flip_vertical"];
    delay = node["frame_delay"];
    fixK1 = node["fix_k1"];
    fixK2 = node["fix_k2"];
    fixK3 = node["fix_k3"];
    fixK4 = node["fix_k4"];
    fixK5 = node["fix_k5"];

    validate();
}

void Calibrate::Settings::validate() {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
        LOG(ERROR) << "Invalid Board size: " << boardSize.width << " " <<
        		boardSize.height;
        goodInput = false;
    }

    if (squareSize <= 10e-6) {
        LOG(ERROR) << "Invalid square size " << squareSize;
        goodInput = false;
    }

    if (nrFrames <= 0) {
        LOG(ERROR) << "Invalid number of frames " << nrFrames;
        goodInput = false;
    }

    flag = 0;
    if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if (aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
    if (fixK1)                  flag |= CALIB_FIX_K1;
    if (fixK2)                  flag |= CALIB_FIX_K2;
    if (fixK3)                  flag |= CALIB_FIX_K3;
    if (fixK4)                  flag |= CALIB_FIX_K4;
    if (fixK5)                  flag |= CALIB_FIX_K5;

    if (useFisheye) {
        // the fisheye model has its own enum, so overwrite the flags
        flag = cv::fisheye::CALIB_FIX_SKEW |
        		cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if (fixK1)                   flag |= cv::fisheye::CALIB_FIX_K1;
        if (fixK2)                   flag |= cv::fisheye::CALIB_FIX_K2;
        if (fixK3)                   flag |= cv::fisheye::CALIB_FIX_K3;
        if (fixK4)                   flag |= cv::fisheye::CALIB_FIX_K4;
        if (calibFixPrincipalPoint)
        	flag |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
    }

    atImageList = 0;
}

Mat Calibrate::_nextImage(size_t cam) {
    Mat result;
    if (cam == 0) {
    	local_->left(result);
    } else if (cam == 1 && local_->isStereo()) {
    	local_->right(result);
    }
    return result;
}

bool Calibrate::Settings::readStringList(const string& filename,
		vector<string>& l) {
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if ( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if ( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for ( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

bool Calibrate::Settings::isListOfImages(const string& filename) {
    string s(filename);
    // Look for file extension
    if ( s.find(".xml") == string::npos && s.find(".yaml") == string::npos &&
    		s.find(".yml") == string::npos )
        return false;
    else
        return true;
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibration(const Calibrate::Settings& s, Size imageSize,
		Mat&  cameraMatrix, Mat& distCoeffs,
		vector<vector<Point2f> > imagePoints, float grid_width,
		bool release_object);


Calibrate::Calibrate(ftl::LocalSource *s, nlohmann::json &config) : local_(s) {
    /*FileStorage fs(cal, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        LOG(ERROR) << "Could not open the configuration file: \"" << cal << "\"";
        return;
    }*/
    // fs["Settings"] >> settings_;
    settings_.read(config);
    // fs.release();

    if (!settings_.goodInput) {
        LOG(ERROR) << "Invalid input detected. Application stopping.";
        return;
    }

    map1_.resize(2);
    map2_.resize(2);

    calibrated_ = _loadCalibration();

    if (calibrated_) {
    	LOG(INFO) << "Calibration loaded from file";
    }
}

bool Calibrate::_loadCalibration() {
	// Capture one frame to get size;
	Mat l, r;
	local_->get(l, r);
	Size img_size = l.size();
	float scale = 1.0f;

    Rect roi1, roi2;

    // reading intrinsic parameters
    FileStorage fs(FTL_LOCAL_CONFIG_ROOT "/intrinsics.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        LOG(WARNING) << "Calibration file not found";
        return false;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        LOG(WARNING) << "Calibration file not found";
        return false;
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q_;

    stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q_,
    		CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2,
    		map1_[0], map2_[0]);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2,
    		map1_[1], map2_[1]);
    return true;
}

bool Calibrate::recalibrate() {
	vector<vector<Point2f>> imagePoints[2];
    Mat cameraMatrix[2], distCoeffs[2];
    Size imageSize[2];

	bool r = _recalibrate(imagePoints, cameraMatrix, distCoeffs, imageSize);

	if (r) calibrated_ = true;

	if (r && local_->isStereo()) {
		int nimages = static_cast<int>(imagePoints[0].size());
		auto squareSize = settings_.squareSize;
		vector<vector<Point3f>> objectPoints;
		objectPoints.resize(nimages);

		for (auto i = 0; i < nimages; i++) {
		    for (auto j = 0; j < settings_.boardSize.height; j++)
		        for (auto  k = 0; k < settings_.boardSize.width; k++)
		            objectPoints[i].push_back(Point3f(k*squareSize,
		            		j*squareSize, 0));
		}

		Mat R, T, E, F;

		LOG(INFO) << "Running stereo calibration...";

		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                cameraMatrix[0], distCoeffs[0],
                cameraMatrix[1], distCoeffs[1],
                imageSize[0], R, T, E, F,
                CALIB_FIX_ASPECT_RATIO +
                CALIB_ZERO_TANGENT_DIST +
                CALIB_USE_INTRINSIC_GUESS +
                CALIB_SAME_FOCAL_LENGTH +
                CALIB_RATIONAL_MODEL +
                CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5));
		LOG(INFO) << "... done with RMS error=" << rms;

		// save intrinsic parameters
		FileStorage fs(FTL_LOCAL_CONFIG_ROOT "/intrinsics.yml", FileStorage::WRITE);
		if (fs.isOpened()) {
		    fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
		        "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		    fs.release();
		} else {
		    LOG(ERROR) << "Error: can not save the intrinsic parameters";
		}

		Mat R1, R2, P1, P2;
		Rect validRoi[2];

		stereoRectify(cameraMatrix[0], distCoeffs[0],
		              cameraMatrix[1], distCoeffs[1],
		              imageSize[0], R, T, R1, R2, P1, P2, Q_,
		              CALIB_ZERO_DISPARITY, 1, imageSize[0],
		              &validRoi[0], &validRoi[1]);

		fs.open(FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml", FileStorage::WRITE);
		if (fs.isOpened()) {
		    fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" <<
		    		P1 << "P2" << P2 << "Q" << Q_;
		    fs.release();
		} else {
		    LOG(ERROR) << "Error: can not save the extrinsic parameters";
		}

		// Precompute maps for cv::remap()
    	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1,
    			imageSize[0], CV_16SC2, map1_[0], map2_[0]);
    	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2,
    			imageSize[0], CV_16SC2, map1_[1], map2_[1]);

	} else {
		int cam = 0;
		if (settings_.useFisheye) {
		    Mat newCamMat;
		    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
		    		cameraMatrix[cam], distCoeffs[cam], imageSize[cam],
		            Matx33d::eye(), newCamMat, 1);
		    cv::fisheye::initUndistortRectifyMap(
		    		cameraMatrix[cam], distCoeffs[cam], Matx33d::eye(),
		    		newCamMat, imageSize[cam],
		            CV_16SC2, map1_[cam], map2_[cam]);
		} else {
		    initUndistortRectifyMap(
		        cameraMatrix[cam], distCoeffs[cam], Mat(),
		        getOptimalNewCameraMatrix(cameraMatrix[cam], distCoeffs[cam],
		        		imageSize[cam], 1, imageSize[cam], 0),
		        imageSize[cam], CV_16SC2, map1_[cam], map2_[cam]);
		}
	}

	return r;
}

bool Calibrate::_recalibrate(vector<vector<Point2f>> *imagePoints,
		Mat *cameraMatrix, Mat *distCoeffs, Size *imageSize) {
	int winSize = 11;  // parser.get<int>("winSize");

    float grid_width = settings_.squareSize * (settings_.boardSize.width - 1);
    bool release_object = false;

    // vector<vector<Point2f> > imagePoints;
    // Mat cameraMatrix, distCoeffs;
    // Size imageSize;
    int mode = CAPTURING;
    double prevTimestamp = 0.0;
    const Scalar RED(0, 0, 255), GREEN(0, 255, 0);

    int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

    if (!settings_.useFisheye) {
        // fast check erroneously fails with high distortions like fisheye
        chessBoardFlags |= CALIB_CB_FAST_CHECK;
    }

    for (;;) {
        Mat view[2];
		local_->get(view[0], view[1]);
        LOG(INFO) << "Grabbing calibration image...";

        if (view[0].empty() || (local_->isStereo() && view[1].empty()) ||
        		imagePoints[0].size() >= (size_t)settings_.nrFrames) {
        	settings_.outputFileName = FTL_LOCAL_CONFIG_ROOT "/cam0.xml";
        	bool r = runCalibration(settings_, imageSize[0],
        					cameraMatrix[0], distCoeffs[0], imagePoints[0],
        					grid_width, release_object);

        	if (local_->isStereo()) {
		    	settings_.outputFileName = FTL_LOCAL_CONFIG_ROOT "/cam1.xml";
		    	r &= runCalibration(settings_, imageSize[1],
		    					cameraMatrix[1], distCoeffs[1], imagePoints[1],
		    					grid_width, release_object);
        	}

        	if (!r && view[0].empty()) {
        		LOG(ERROR) << "Not enough frames to calibrate";
        		return false;
        	} else if (r) {
        		LOG(INFO) << "Calibration successful";
        		break;
        	}
        }

        imageSize[0] = view[0].size();  // Format input image.
        imageSize[1] = view[1].size();
        // if( settings_.flipVertical )    flip( view[cam], view[cam], 0 );

        vector<Point2f> pointBuf[2];
        bool found1, found2;
		found1 = findChessboardCorners(view[0], settings_.boardSize,
				pointBuf[0], chessBoardFlags);
		found2 = !local_->isStereo() || findChessboardCorners(view[1],
				settings_.boardSize, pointBuf[1], chessBoardFlags);

        if (found1 && found2 &&
        		local_->getTimestamp()-prevTimestamp > settings_.delay) {
			prevTimestamp = local_->getTimestamp();
            // improve the found corners' coordinate accuracy for chessboard
                Mat viewGray;
                cvtColor(view[0], viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf[0], Size(winSize, winSize),
                    Size(-1, -1), TermCriteria(
                    	TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001));
                imagePoints[0].push_back(pointBuf[0]);

                if (local_->isStereo()) {
                    cvtColor(view[1], viewGray, COLOR_BGR2GRAY);
                    cornerSubPix(viewGray, pointBuf[1], Size(winSize, winSize),
                        Size(-1, -1), TermCriteria(
                        	TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001));
					imagePoints[1].push_back(pointBuf[1]);
                }

                // Draw the corners.
                drawChessboardCorners(view[0], settings_.boardSize,
                		Mat(pointBuf[0]), found1);
                if (local_->isStereo()) drawChessboardCorners(view[1],
                		settings_.boardSize, Mat(pointBuf[1]), found2);
        } else {
        	LOG(WARNING) << "No calibration pattern found";
        }

        imshow("Left", view[0]);
        if (local_->isStereo()) imshow("Right", view[1]);
        char key = static_cast<char>(waitKey(50));

        if (key  == 27)
            break;
    }

	return true;
}

bool Calibrate::undistort(cv::Mat &l, cv::Mat &r) {
	local_->get(l, r);
	if (!calibrated_) return false;
	if (l.empty()) return false;
	remap(l, l, map1_[0], map2_[0], INTER_LINEAR);
	if (local_->isStereo()) remap(r, r, map1_[1], map2_[1], INTER_LINEAR);
	return true;
}

bool Calibrate::rectified(cv::Mat &l, cv::Mat &r) {
	return undistort(l, r);
}

bool Calibrate::isCalibrated() {
	return calibrated_;
}

//! [compute_errors]
static double computeReprojectionErrors(
		const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints,
		const vector<Mat>& rvecs, const vector<Mat>& tvecs,
		const Mat& cameraMatrix , const Mat& distCoeffs,
		vector<float>& perViewErrors, bool fisheye) {
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        if (fisheye) {
            cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i],
            		tvecs[i], cameraMatrix, distCoeffs);
        } else {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix,
            		distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = static_cast<float>(std::sqrt(err*err/n));
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize,
		vector<Point3f>& corners, Calibrate::Settings::Pattern patternType) {
    corners.clear();

    switch (patternType) {
    case Calibrate::Settings::CHESSBOARD:
    case Calibrate::Settings::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i)
            for (int j = 0; j < boardSize.width; ++j)
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Calibrate::Settings::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(Point3f((2*j + i % 2)*squareSize,
                		i*squareSize, 0));
        break;
    default:
        break;
    }
}
//! [board_corners]
static bool _runCalibration(const Calibrate::Settings& s, Size& imageSize,
		Mat& cameraMatrix, Mat& distCoeffs,
		vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs,
		vector<Mat>& tvecs, vector<float>& reprojErrs,  double& totalAvgErr,
		vector<Point3f>& newObjPoints, float grid_width, bool release_object) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = s.aspectRatio;

    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0],
    		Calibrate::Settings::CHESSBOARD);
    objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x +
    		grid_width;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    // Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = cv::fisheye::calibrate(objectPoints, imagePoints, imageSize,
        		cameraMatrix, distCoeffs, _rvecs, _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for (int i = 0; i < static_cast<int>(objectPoints.size()); i++) {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        int iFixedPoint = -1;
        if (release_object)
            iFixedPoint = s.boardSize.width - 1;

        rms = calibrateCamera(objectPoints, imagePoints, imageSize,
                                cameraMatrix, distCoeffs, rvecs, tvecs,
                                s.flag | CALIB_USE_LU);
    }

    LOG(INFO) << "Re-projection error reported by calibrateCamera: "<< rms;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs,
    		tvecs, cameraMatrix, distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

//! [run_and_save]
bool runCalibration(const Calibrate::Settings& s, Size imageSize,
		Mat& cameraMatrix, Mat& distCoeffs,
		vector<vector<Point2f> > imagePoints, float grid_width,
		bool release_object) {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = _runCalibration(s, imageSize, cameraMatrix, distCoeffs,
    		imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr, newObjPoints,
    		grid_width, release_object);
    LOG(INFO) << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr;

    return ok;
}
//! [run_and_save]
