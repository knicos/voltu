#include "align.hpp"

#include <loguru.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using std::map;
using std::string;
using cv::Mat;
using std::vector;
using cv::Point2f;
using cv::Size;

struct Rec4f {
	float left;
	float right;
	float top;
	float bottom;
};

bool loadIntrinsicsMap(const std::string &ifile, const cv::Size &imageSize, Mat &map1, Mat &map2, Mat &cameraMatrix, float scale) {
    using namespace cv;

    FileStorage fs;

    // reading intrinsic parameters
        fs.open((ifile).c_str(), FileStorage::READ);
        if (!fs.isOpened()) {
            LOG(WARNING) << "Could not open intrinsics file : " << ifile;
            return false;
        }
        
        LOG(INFO) << "Intrinsics from: " << ifile;


    Mat D1;
    fs["M"] >> cameraMatrix;
    fs["D"] >> D1;

    //cameraMatrix *= scale;

	initUndistortRectifyMap(cameraMatrix, D1, Mat::eye(3,3, CV_64F), cameraMatrix, imageSize, CV_16SC2,
    		map1, map2);

    return true;
}

inline bool hasOption(const map<string, string> &options, const std::string &opt) {
    return options.find(opt) != options.end();
}

inline std::string getOption(map<string, string> &options, const std::string &opt) {
    auto str = options[opt];
    return str.substr(1,str.size()-2);
}

static const float kPI = 3.14159f;

static float calculateZRotation(const vector<Point2f> &points, Size &boardSize) {
    Point2f tl = points[boardSize.width * (boardSize.height / 2)];
    Point2f tr = points[boardSize.width * (boardSize.height / 2) + boardSize.width-1];

    float dx = tr.x - tl.x;
    float dy = tr.y - tl.y;
    float angle = atan2(dy,  dx) * (180.0f / kPI);
    return angle;
}

static Point2f parallaxDistortion(const vector<Point2f> &points, Size &boardSize) {
    Point2f tl = points[0];
    Point2f tr = points[boardSize.width-1];
    Point2f bl = points[(boardSize.height-1)*boardSize.width];
    Point2f br = points[points.size()-1];

    float dx1 = tr.x - tl.x;
    float dx2 = br.x - bl.x;
    float ddx = dx1 - dx2;

    float dy1 = bl.y - tl.y;
    float dy2 = br.y - tr.y;
    float ddy = dy1 - dy2;

    return Point2f(ddx, ddy);
}

static float distanceTop(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
    Point2f tl = points[0];
    Point2f tr = points[boardSize.width-1];

    float pixSize = tr.x - tl.x;
    float mmSize = boardSize.width * squareSize;
    float focal = camMatrix.at<double>(0,0);

    return ((mmSize / pixSize) * focal) / 1000.0f;
}

static float distanceBottom(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
    Point2f bl = points[(boardSize.height-1)*boardSize.width];
    Point2f br = points[points.size()-1];

    float pixSize = br.x - bl.x;
    float mmSize = boardSize.width * squareSize;
    float focal = camMatrix.at<double>(0,0);

    return ((mmSize / pixSize) * focal) / 1000.0f;
}

static float distanceLeft(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
    Point2f bl = points[(boardSize.height-1)*boardSize.width];
    Point2f tl = points[0];

    float pixSize = bl.y - tl.y;
    float mmSize = boardSize.height * squareSize;
    float focal = camMatrix.at<double>(0,0);

    return ((mmSize / pixSize) * focal) / 1000.0f;
}

static float distanceRight(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
    Point2f tr = points[boardSize.width-1];
    Point2f br = points[points.size()-1];

    float pixSize = br.y - tr.y;
    float mmSize = boardSize.height * squareSize;
    float focal = camMatrix.at<double>(0,0);

    return ((mmSize / pixSize) * focal) / 1000.0f;
}

static Rec4f distances(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
	return {
		-distanceLeft(camMatrix, points, boardSize, squareSize),
		-distanceRight(camMatrix, points, boardSize, squareSize),
		-distanceTop(camMatrix, points, boardSize, squareSize),
		-distanceBottom(camMatrix, points, boardSize, squareSize)
	};
}

static float distance(const Mat &camMatrix, const vector<Point2f> &points, Size &boardSize, float squareSize) {
    Point2f tl = points[boardSize.width * (boardSize.height / 2)];
    Point2f tr = points[boardSize.width * (boardSize.height / 2) + boardSize.width-1];

    float pixSize = tr.x - tl.x;
    float mmSize = boardSize.width * squareSize;
    float focal = camMatrix.at<double>(0,0);

    return ((mmSize / pixSize) * focal) / 1000.0f;
}

static Point2f diffY(const vector<Point2f> &pointsA, const vector<Point2f> &pointsB, Size &boardSize) {
    Point2f tlA = pointsA[boardSize.width * (boardSize.height / 2)];
    Point2f trA = pointsA[boardSize.width * (boardSize.height / 2) + boardSize.width-1];

    Point2f tlB = pointsB[boardSize.width * (boardSize.height / 2)];
    Point2f trB = pointsB[boardSize.width * (boardSize.height / 2) + boardSize.width-1];

    float d1 = tlA.y - tlB.y;
    float d2 = trA.y - trB.y;

    return Point2f(d1,d2);
}

static const float kDistanceThreshold = 0.005f;


static void showAnaglyph(const Mat &frame_l, const Mat &frame_r, Mat &img3d) {
    using namespace cv;

    float data[] = {0.299f, 0.587f, 0.114f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.299, 0.587, 0.114};
    Mat m(2, 9, CV_32FC1, data);

    //Mat img3d;

    img3d = Mat(frame_l.size(), CV_8UC3);
  
    for (int y=0; y<img3d.rows; y++) {
        unsigned char *row3d = img3d.ptr(y);
        const unsigned char *rowL = frame_l.ptr(y);
        const unsigned char *rowR = frame_r.ptr(y);

        for (int x=0; x<img3d.cols*3; x+=3) {
            const uchar lb = rowL[x+0];
            const uchar lg = rowL[x+1];
            const uchar lr = rowL[x+2];
            const uchar rb = rowR[x+0];
            const uchar rg = rowR[x+1];
            const uchar rr = rowR[x+2];

            row3d[x+0] = lb*m.at<float>(0,6) + lg*m.at<float>(0,7) + lr*m.at<float>(0,8) + rb*m.at<float>(1,6) + rg*m.at<float>(1,7) + rr*m.at<float>(1,8);
            row3d[x+1] = lb*m.at<float>(0,3) + lg*m.at<float>(0,4) + lr*m.at<float>(0,5) + rb*m.at<float>(1,3) + rg*m.at<float>(1,4) + rr*m.at<float>(1,5);
            row3d[x+2] = lb*m.at<float>(0,0) + lg*m.at<float>(0,1) + lr*m.at<float>(0,2) + rb*m.at<float>(1,0) + rg*m.at<float>(1,1) + rr*m.at<float>(1,2);
        }
    }

    //imshow("Anaglyph", img3d);
    //return img3d;
}

void ftl::calibration::align(map<string, string> &opt) {
    using namespace cv;

    float squareSize = 36.0f;

    VideoCapture camA(0);
    VideoCapture camB(1);

    if (!camA.isOpened() || !camB.isOpened()) {
        LOG(ERROR) << "Could not open a camera device";
        return;
    }

    camA.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // TODO Use settings
	camA.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    camB.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	camB.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    Mat map1, map2, cameraMatrix;
    Size imgSize(1280,720);
    loadIntrinsicsMap((hasOption(opt, "profile")) ? getOption(opt,"profile") : "./panasonic.yml", imgSize, map1, map2, cameraMatrix, 1.0f);

    Size boardSize(9,6);

#if CV_VERSION_MAJOR >= 4
	int chessBoardFlags = CALIB_CB_NORMALIZE_IMAGE;
#else
	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

	if (!settings.useFisheye) {
		// fast check erroneously fails with high distortions like fisheye
		chessBoardFlags |= CALIB_CB_FAST_CHECK;
	}
#endif

    int step = 0;
    bool anaglyph = true;

    while (true) {
        Mat frameA, fA;
        Mat frameB, fB;

        camA.grab();
        camB.grab();
        camA.retrieve(frameA);
        camB.retrieve(frameB);

        remap(frameA, fA, map1, map2, INTER_LINEAR);
        remap(frameB, fB, map1, map2, INTER_LINEAR);

        // Get the chessboard
        vector<Point2f> pointBufA;
        vector<Point2f> pointBufB;
		bool foundA, foundB;
        foundA = findChessboardCornersSB(fA, boardSize,
				pointBufA, chessBoardFlags);
        foundB = findChessboardCornersSB(fB, boardSize,
				pointBufB, chessBoardFlags);

    // Step 1: Position cameras correctly with respect to chessboard
    //      - print distance estimate etc

    // Step 2: Show individual camera tilt degrees with left / right indicators

    // Show also up down tilt perspective error

    // Step 3: Display current baseline in mm

        if (foundA) {
            // Draw the corners.
			//drawChessboardCorners(fA, boardSize,
			//		Mat(pointBufA), foundA);
        }

        if (foundB) {
            // Draw the corners.
			//drawChessboardCorners(fB, boardSize,
			//		Mat(pointBufB), foundB);
        }

        Mat anag;
        showAnaglyph(fA, fB, anag);

        if (foundA) {
			Rec4f dists = distances(cameraMatrix, pointBufA, boardSize, squareSize);
			//Rec4f angs = angles(pointBufA, boardSize);

			// TODO Check angles also...
			bool lrValid = std::abs(dists.left-dists.right) <= kDistanceThreshold;
			bool tbValid = std::abs(dists.top-dists.bottom) <= kDistanceThreshold;
			bool distValid = lrValid & tbValid;
			bool tiltUp = dists.top < dists.bottom && !tbValid;
			bool tiltDown = dists.top > dists.bottom && !tbValid;
			bool rotLeft = dists.left > dists.right && !lrValid;
			bool rotRight = dists.left < dists.right && !lrValid;

			float d = (dists.left + dists.right + dists.top + dists.bottom) / 4.0f;

			// TODO Draw lines
            Point2f bl = pointBufA[(boardSize.height-1)*boardSize.width];
            Point2f tl = pointBufA[0];
            Point2f tr = pointBufA[boardSize.width-1];
            Point2f br = pointBufA[pointBufA.size()-1];


            line(anag, tl, tr, (!lrValid && tiltUp) ? Scalar(0,0,255) : Scalar(0,255,0));
            line(anag, bl, br, (!lrValid && tiltDown) ? Scalar(0,0,255) : Scalar(0,255,0));
            line(anag, tl, bl, (!tbValid && rotLeft) ? Scalar(0,0,255) : Scalar(0,255,0));
            line(anag, tr, br, (!tbValid && rotRight) ? Scalar(0,0,255) : Scalar(0,255,0));

            //fA = fA(Rect(tl.x - 100, tl.y- 100, br.x-tl.x + 200, br.y-tl.y + 200));

			// Show distance error between cameras

			// Show estimated baseline

            //if (step == 0) {
            //    Point2f pd = parallaxDistortion(pointBufA, boardSize);
            //    putText(fA, string("Distort: ") + std::to_string(pd.x) + string(",") + std::to_string(pd.y), Point(10,50), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
            //} else if (step == 1) {
                //float d = distance(cameraMatrix, pointBufA, boardSize, squareSize);
                //putText(anag, string("Distance: ") + std::to_string(-d) + string("m"), Point(10,50), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
           // } else if (step == 2) {
                //float angle = calculateZRotation(pointBufA, boardSize) - 180.0f;
                //putText(anag, string("Angle: ") + std::to_string(angle), Point(10,150), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
            //} else if (step == 3) {
            //    Point2f vd = diffY(pointBufA, pointBufB, boardSize);
            //    putText(fA, string("Vertical: ") + std::to_string(vd.x) + string(",") + std::to_string(vd.y), Point(10,200), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
           // }

            if (foundB) {
                //if (step == 0) {
                    //Point2f pd = parallaxDistortion(pointBufB, boardSize);
                    //putText(fB, string("Distort: ") + std::to_string(pd.x) + string(",") + std::to_string(pd.y), Point(10,50), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
                //} else if (step == 1) {
                    //float d = distance(cameraMatrix, pointBufB, boardSize, squareSize);
                    //putText(fB, string("Distance: ") + std::to_string(-d) + string("m"), Point(10,100), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
                //} else if (step == 2) {
                    //float angle = calculateZRotation(pointBufB, boardSize) - 180.0f;
                    //putText(fB, string("Angle: ") + std::to_string(angle), Point(10,150), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 3);
                //}

                Rec4f dists = distances(cameraMatrix, pointBufB, boardSize, squareSize);
                //Rec4f angs = angles(pointBufA, boardSize);

                // TODO Check angles also...
                bool lrValid = std::abs(dists.left-dists.right) <= kDistanceThreshold;
                bool tbValid = std::abs(dists.top-dists.bottom) <= kDistanceThreshold;
                bool distValid = lrValid & tbValid;
                bool tiltUp = dists.top < dists.bottom && !tbValid;
                bool tiltDown = dists.top > dists.bottom && !tbValid;
                bool rotLeft = dists.left > dists.right && !lrValid;
                bool rotRight = dists.left < dists.right && !lrValid;

                float d = (dists.left + dists.right + dists.top + dists.bottom) / 4.0f;

                // TODO Draw lines
                Point2f bbl = pointBufB[(boardSize.height-1)*boardSize.width];
                Point2f btl = pointBufB[0];
                Point2f btr = pointBufB[boardSize.width-1];
                Point2f bbr = pointBufB[pointBufB.size()-1];


                line(anag, btl, btr, (!lrValid && tiltUp) ? Scalar(0,0,255) : Scalar(0,255,0));
                line(anag, bbl, bbr, (!lrValid && tiltDown) ? Scalar(0,0,255) : Scalar(0,255,0));
                line(anag, btl, bbl, (!tbValid && rotLeft) ? Scalar(0,0,255) : Scalar(0,255,0));
                line(anag, btr, bbr, (!tbValid && rotRight) ? Scalar(0,0,255) : Scalar(0,255,0));

                float baseline1 = std::abs(tl.x - btl.x);
                float baseline2 = std::abs(tr.x - btr.x);
                float baseline3 = std::abs(bl.x - bbl.x);
                float baseline4 = std::abs(br.x - bbr.x);
                float boardWidth = (std::abs(tl.x-tr.x) + std::abs(btl.x - btr.x)) / 2.0f;
                float baseline = ((baseline1 + baseline2 + baseline3 + baseline4) / 4.0f) / boardWidth * (boardSize.width*squareSize);

                putText(anag, string("Baseline: ") + std::to_string(baseline) + string("mm"), Point(10,150), FONT_HERSHEY_PLAIN, 2.0, Scalar(0,255,0), 2);
            }

        }

        /*if (anaglyph) {
            showAnaglyph(fA,fB);
        } else {
            imshow("Left", fA);
            imshow("Right", fB);
        }*/
        imshow("Anaglyph", anag);

		char key = static_cast<char>(waitKey(20));
		if (key  == 27)
			break;
        if (key == 32) anaglyph = !anaglyph;
    }
}