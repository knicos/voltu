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

	ftlCheck(ftlSetFrameRate(s, 20.0f));

	// Two test frames, red and green
	cv::Mat test_image1(720, 1280, CV_8UC4, cv::Scalar(0,0,255,255));
	cv::Mat test_image2(720, 1280, CV_8UC4, cv::Scalar(0,255,0,255));

	ftlCheck(ftlIntrinsicsWriteLeft(s, 0, 1280, 720, 300.0f, -1280.0f/2.0f, -720.0f/2.0f, 0.1f, 0.1f, 8.0f));

	// Write a number of frames, alternating images
	for (int i=0; i<100; ++i) {
		if (i&1) {
			ftlCheck(ftlImageWrite(s, 0, FTLCHANNEL_Colour, FTLIMAGE_BGRA, test_image2.step, test_image2.data));
		} else {
			ftlCheck(ftlImageWrite(s, 0, FTLCHANNEL_Colour, FTLIMAGE_BGRA, test_image1.step, test_image1.data));
		}

		ftlCheck(ftlNextFrame(s));
	}
	

	ftlCheck(ftlDestroyStream(s));

	return 0;
}
