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

#include <glog/logging.h>

using namespace cv;
using namespace std;

using ftl::Calibrate;


void Calibrate::Settings::write(FileStorage& fs) const                        //Write serialization for this class
{
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
              //<< "Write_outputFileName"  << outputFileName

              //<< "Show_UndistortedImage" << showUndistorsed

              << "Input_FlipAroundHorizontalAxis" << flipVertical
              << "Input_Delay" << delay
              //<< "Input" << input
       << "}";
}
void Calibrate::Settings::read(const FileNode& node)                          //Read serialization for this class
{
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"]  >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_gridPoints"] >> writeGrid;
    //node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    //node["Show_UndistortedImage"] >> showUndistorsed;
    //node["Input"] >> input;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}
void Calibrate::Settings::validate()
{
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0)
    {
        LOG(ERROR) << "Invalid Board size: " << boardSize.width << " " << boardSize.height;
        goodInput = false;
    }
    if (squareSize <= 10e-6)
    {
        LOG(ERROR) << "Invalid square size " << squareSize;
        goodInput = false;
    }
    if (nrFrames <= 0)
    {
        LOG(ERROR) << "Invalid number of frames " << nrFrames;
        goodInput = false;
    }

    flag = 0;
    if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= CALIB_FIX_K1;
    if(fixK2)                  flag |= CALIB_FIX_K2;
    if(fixK3)                  flag |= CALIB_FIX_K3;
    if(fixK4)                  flag |= CALIB_FIX_K4;
    if(fixK5)                  flag |= CALIB_FIX_K5;

    if (useFisheye) {
        // the fisheye model has its own enum, so overwrite the flags
        flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if(fixK1)                   flag |= fisheye::CALIB_FIX_K1;
        if(fixK2)                   flag |= fisheye::CALIB_FIX_K2;
        if(fixK3)                   flag |= fisheye::CALIB_FIX_K3;
        if(fixK4)                   flag |= fisheye::CALIB_FIX_K4;
        if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
    }

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING)
    {
        LOG(ERROR) << " Camera calibration mode does not exist: " << patternToUse;
        goodInput = false;
    }
    atImageList = 0;

}
Mat Calibrate::_nextImage(size_t cam)
{
    Mat result;
    if (cam == 0) {
    	local_->left(result);
    } else if (cam == 1 && local_->isStereo()) {
    	local_->right(result);
    }
    return result;
}

bool Calibrate::Settings::readStringList( const string& filename, vector<string>& l )
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

bool Calibrate::Settings::isListOfImages( const string& filename)
{
    string s(filename);
    // Look for file extension
    if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
        return false;
    else
        return true;
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Calibrate::Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object);


Calibrate::Calibrate(ftl::LocalSource *s, const std::string &cal) : local_(s) {
    FileStorage fs(cal, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        LOG(ERROR) << "Could not open the configuration file: \"" << cal << "\"";
        return;
    }
    //fs["Settings"] >> settings_;
    settings_.read(fs["Settings"]);
    fs.release();
    
    if (!settings_.goodInput)
    {
        LOG(ERROR) << "Invalid input detected. Application stopping.";
        return;
    }
    
    map1_.resize(2);
    map2_.resize(2);
    
    calibrated_ = false;
}

bool Calibrate::recalibrate() {
	bool r = _recalibrate(0);
	if (local_->isStereo()) r &= _recalibrate(1);
	
	if (r) calibrated_ = true;
	
	return r;
}

bool Calibrate::_recalibrate(size_t cam) {
	// TODO WHAT IS WINSIZE!!
	int winSize = 11; //parser.get<int>("winSize");

    float grid_width = settings_.squareSize * (settings_.boardSize.width - 1);
    bool release_object = false;

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = CAPTURING;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    
    settings_.outputFileName = string(FTL_CONFIG_ROOT "/calib_cam") + std::to_string(cam) + ".yml";

    //! [get_input]
    for(;;)
    {
        Mat view;
        //bool blinkOutput = false;

        view = _nextImage(cam);
        LOG(INFO) << "Grabbing calibration image...";

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if( mode == CAPTURING && imagePoints.size() >= (size_t)settings_.nrFrames )
        {
          if(runCalibrationAndSave(settings_, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
                                   release_object)) {
              LOG(INFO) << "Calibration completed";
              mode = CALIBRATED;
        }  else
              mode = DETECTION;
        }
        if(view.empty())          // If there are no more images stop the loop
        {
        	LOG(INFO) << "More calibration images are required";
            // if calibration threshold was not reached yet, calibrate now
            if( mode != CALIBRATED && !imagePoints.empty() )
                runCalibrationAndSave(settings_, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
                                      release_object);
            break;
        }
        //! [get_input]

        imageSize = view.size();  // Format input image.
        if( settings_.flipVertical )    flip( view, view, 0 );

        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;

        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if(!settings_.useFisheye) {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        switch( settings_.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, settings_.boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, settings_.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, settings_.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }
        //! [find_pattern]
        //! [pattern_found]
        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( settings_.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(winSize,winSize),
                        Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (clock() - prevTimestamp > settings_.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                   // blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, settings_.boardSize, Mat(pointBuf), found );
        } else {
        	LOG(WARNING) << "No calibration pattern found";
        }
        //! [pattern_found]
        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        /*string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);*/
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        /*if( mode == CALIBRATED && settings_.showUndistorsed )
        {
            Mat temp = view.clone();
            if (settings_.useFisheye)
              cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
            else
              cv::undistort(temp, view, cameraMatrix, distCoeffs);
        }*/
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("Image View", view);
        char key = (char)waitKey(settings_.delay);

        if( key  == 27 )
            break;

        /*if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }*/
        //! [await_input]
        if (mode == CALIBRATED) break;
    }
    
    if (mode != CALIBRATED) return false;
    
    Mat view, rview;

    if (settings_.useFisheye)
    {
        Mat newCamMat;
        fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                            Matx33d::eye(), newCamMat, 1);
        fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                         CV_16SC2, map1_[cam], map2_[cam]);
    }
    else
    {
        initUndistortRectifyMap(
            cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
            CV_16SC2, map1_[cam], map2_[cam]);
    }
	return true;
}

bool Calibrate::undistort(cv::Mat &l, cv::Mat &r) {
	local_->get(l,r);
	if (!calibrated_) return false;
	if (l.empty()) return false;
	remap(l, l, map1_[0], map2_[0], INTER_LINEAR);
	if (local_->isStereo()) remap(r, r, map1_[1], map2_[1], INTER_LINEAR);
	return true;
}

bool Calibrate::rectified(cv::Mat &l, cv::Mat &r) {
	return undistort(l,r);
}

bool Calibrate::isCalibrated() {
	return calibrated_;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Calibrate::Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Calibrate::Settings::CHESSBOARD:
    case Calibrate::Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Calibrate::Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}
//! [board_corners]
static bool runCalibration( Calibrate::Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr, vector<Point3f>& newObjPoints,
                            float grid_width, bool release_object)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        int iFixedPoint = -1;
        if (release_object)
            iFixedPoint = s.boardSize.width - 1;
        rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                                cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                                s.flag | CALIB_USE_LU);
    }

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[s.boardSize.width - 1] << endl;
        cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Calibrate::Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, const vector<Point3f>& newObjPoints )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );
    
    LOG(INFO) << "Saving calibration to " << s.outputFileName;

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    if( s.writeGrid && !newObjPoints.empty() )
    {
        fs << "grid_points" << newObjPoints;
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Calibrate::Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr, newObjPoints, grid_width, release_object);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;
         
	//return ok;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, newObjPoints);
    return ok;
}
//! [run_and_save]
