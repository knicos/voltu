#include "mutual_information.hpp"
#include "../util.hpp"
#include "../util_opencv.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <cuda_runtime.h>

/**
 * Mutual Information matching cost.
 *
 * Hirschmüller, H. (2008). Stereo processing by Semiglobal Matching and Mutual
 * Information. IEEE Transactions on Pattern Analysis and Machine Intelligence,
 * 30(2), 328–341. https://doi.org/10.1109/TPAMI.2007.1166
 *
 * Kim, J., Kolmogorov, V., & Zabih, R. (2003). Visual correspondence using
 * energy minimization and mutual information. Proceedings of the IEEE
 * International Conference on Computer Vision.
 * https://doi.org/10.1109/iccv.2003.1238463
 */

/** Calculates intensity distribution. Assumes input in 8 bit grayscale. */
void calculate_intensity_distribution(const cv::Mat &im1, const cv::Mat &im2, cv::Mat &out, const cv::Mat &mask) {

	if (im1.size() != im2.size()) { throw std::exception(); }
	if (im1.type() != im2.type()) { throw std::exception(); }
	if (im1.type() != CV_8UC1) { throw std::exception(); }

	if (!mask.empty() && im1.size() != mask.size()) { throw std::exception(); }
	if (!mask.empty() && mask.type() != CV_32FC1) { throw std::exception(); }

	const int n = std::numeric_limits<uchar>::max() + 1;
	out.create(cv::Size(n, n), CV_32FC1);
	out.setTo(0.0f);

	for (int y = 0; y < im1.rows; y++) {
		for (int x = 0; x < im1.cols; x++) {
			if (!mask.empty() && mask.at<float>(y,x) == 0.0f) { continue; }

			const uchar i1 = im1.at<uchar>(y,x);
			const uchar i2 = im2.at<uchar>(y,x);
			out.at<float>(i1,i2) += 1.0f;
			out.at<float>(i2,i1) += 1.0f;
		}
	}

	out /= float(im1.total());
}

/** Sum over columns (data term h1) **/
void calculate_h1(const cv::Mat &P, cv::Mat &out) {
	out.create(cv::Size(P.cols, 1), CV_32FC1);
	for (int x = 0; x < P.cols; x++) {
		float &sum = out.at<float>(x);
		sum = 0.0;
		for (int y = 0; y < P.rows; y++) { sum += P.at<float>(y,x); }
	}
}

/** Sum over rows (data term h2) **/
void calculate_h2(const cv::Mat &P, cv::Mat &out) {
	out.create(cv::Size(P.rows, 1), CV_32FC1);
	for (int y = 0; y < P.rows; y++) {
		float &sum = out.at<float>(y);
		sum = 0.0;
		for (int x = 0; x < P.cols; x++) { sum += P.at<float>(y,x); }
	}
}

/** Data term h12 */
void calculate_h12(const cv::Mat &P, cv::Mat &out, int ksize=7, double sigma=1.0) {
	if (P.empty() || P.type() != CV_32FC1) { throw std::exception(); }

	// possible numerical issues? should also be less than 1/(width*height) of
	// original image.
	static float e = 0.00000001;
	cv::Mat tmp;

	P.copyTo(out);
	out.setTo(e, P == 0.0f);

	cv::GaussianBlur(out, tmp, cv::Size(ksize,ksize), sigma);
	cv::log(tmp, tmp);
	tmp *= -1.0f;

	cv::GaussianBlur(tmp, out, cv::Size(ksize,ksize), sigma);
}

/** Warp (remap) image using disparity map.
 *
 * @param	disparity		disparity map (CV_32FC1)
 * @param	im				input image
 * @param	out				result, same type as im
 * @param	mapx			map for x
 * @param	mapy			map for y, re-calculates if not provided (mapy.empty() == true)
 * @param	interpolation	interpolation method, see cv::remap (default linear)
 * @param	sign			disparity sign
 */
void remap_disparity(const cv::Mat &disparity, const cv::Mat &im, cv::Mat &out,
		cv::Mat &mapx, cv::Mat &mapy, int interpolation=1, int sign=-1) {

	if (mapy.size() != im.size() || mapy.type() != CV_32FC1) {
		mapy.create(disparity.size(), CV_32FC1);
		for (int y = 0; y < disparity.rows; y++) {
			float *ptr = mapy.ptr<float>(y);
			for (int x = 0; x < disparity.cols; x++) { ptr[x] = float(y); }
		}
	}

	mapx.create(disparity.size(), CV_32FC1);
	for (int y = 0; y < disparity.rows; y++) {
		float *ptr = mapx.ptr<float>(y);
		const float *d = disparity.ptr<float>(y);
		for (int x = 0; x < disparity.cols; x++) {
			if (d[x] == 0.0f) {
				ptr[x] = NAN;
			}
			else {
				ptr[x] = float(x) + d[x]*float(sign);
			}
		}
	}

	cv::remap(im, out, mapx, mapy, interpolation, cv::BORDER_CONSTANT);
}

void MutualInformationMatchingCost::set(const cv::Mat &l, const cv::Mat &r, const cv::Mat &disparity) {
	cv::Mat P;

	if (l.type() != r.type()) { throw std::exception(); }
	remap_disparity(disparity, r, r_warp_, mapx_, mapy_);

	mat2gray(l, l_);
	mat2gray(r, r_);
	cv::cvtColor(r_warp_, r_warp_, cv::COLOR_BGR2GRAY);

	calculate_intensity_distribution(l_.toMat(), r_warp_, P, disparity);

	cv::Mat h12;
	cv::Mat h1;
	cv::Mat h2;

	calculate_h12(P, h12);
	calculate_h1(P, h1);
	calculate_h2(P, h2);

	#if USE_GPU
	h12_.toGpuMat().upload(h12);
	h1_.toGpuMat().upload(h1);
	h2_.toGpuMat().upload(h2);
	#else
	h12.copyTo(h12_.toMat());
	h1.copyTo(h1_.toMat());
	h2.copyTo(h2_.toMat());
	#endif
}
