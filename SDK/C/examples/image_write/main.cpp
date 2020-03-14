#include <ftl/ftl.h>
#include <opencv2/core/mat.hpp>
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <Eigen/Eigen>

static void ftlCheck(ftlError_t err) {
	if (err != FTLERROR_OK) {
		LOG(ERROR) << "FTL Stream Error: " << err;
		exit(-1);
	}
}

int main(int argc, char **argv) {
	ftlStream_t s = ftlCreateWriteStream("./out.ftl");
	if (!s) ftlCheck(ftlGetLastStreamError(s));

	// Two test images, red and green
	cv::Mat test_image1(720, 1280, CV_8UC4, cv::Scalar(0,0,255,255));
	cv::Mat test_image2(720, 1280, CV_8UC4, cv::Scalar(0,255,0,255));

	// Two test depth maps
	cv::Mat test_depth1(720, 1280, CV_32F, cv::Scalar(3.0f));
	cv::Mat test_depth2(720, 1280, CV_32F, cv::Scalar(2.0f));

	// Write red image
	ftlCheck(ftlIntrinsicsWriteLeft(s, 0, 1280, 720, 300.0f, -1280.0f/2.0f, -720.0f/2.0f, 0.1f, 0.1f, 8.0f));
	ftlCheck(ftlImageWrite(s, 0, FTLCHANNEL_Colour, FTLIMAGE_BGRA, test_image1.step, test_image1.data));

	// Write green image
	ftlCheck(ftlIntrinsicsWriteLeft(s, 1, 1280, 720, 300.0f, -1280.0f/2.0f, -720.0f/2.0f, 0.1f, 0.1f, 8.0f));
	ftlCheck(ftlImageWrite(s, 1, FTLCHANNEL_Colour, FTLIMAGE_BGRA, test_image2.step, test_image2.data));

	// Write depth images
	ftlCheck(ftlImageWrite(s, 0, FTLCHANNEL_Depth, FTLIMAGE_FLOAT, test_depth1.step, test_depth1.data));
	ftlCheck(ftlImageWrite(s, 1, FTLCHANNEL_Depth, FTLIMAGE_FLOAT, test_depth2.step, test_depth2.data));

	// Set pose for second source
	Eigen::Translation3f trans(1.0f, 0.5f, 0.0f);
	Eigen::Affine3f t(trans);
	Eigen::Matrix4f viewPose = t.matrix();
	ftlCheck(ftlPoseWrite(s, 1, viewPose.data()));

	ftlCheck(ftlDestroyStream(s));

	return 0;
}
