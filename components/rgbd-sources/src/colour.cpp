#include "colour.hpp"

#include <vector>
#include <tuple>

using std::vector;

/*static void gammaCorrection(const cv::Mat &img, const double gamma_){
	using namespace cv;

    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);

    Mat res = img.clone();
    LUT(img, lookUpTable, img);
}*/

static const std::vector<std::tuple<uchar,uchar,uchar>> kelvin_table = {
    {255,56,0},		// 1000
    {255,109,0},		// 1500
    {255,137,18},		// 2000
    {255,161,72},		// 2500
    {255,180,107},	// 3000
    {255,196,137},	// 3500
    {255,209,163},	// 4000
    {255,219,186},	// 4500
    {255,228,206},	// 5000
    {255,236,224},	// 5500
    {255,243,239},	// 6000
    {255,249,253},	// 6500
    {245,243,255},	// 7000
    {235,238,255},	// 7500
    {227,233,255},	// 8000
    {220,229,255},	// 8500
    {214,225,255},	// 9000
    {208,222,255},	// 9500
    {204,219,255}};	// 10000

template <int C>
inline float kelvinFactor(int temp) {
	int index = (temp / 500) - 2;
	if (index >= (int)kelvin_table.size()) index = kelvin_table.size();
	else if (index < 0) index = 0;
	return (float)std::get<C>(kelvin_table[index]) / 255.0f;
}

// TODO: Implement CUDA version
void ftl::rgbd::colourCorrection(cv::Mat &img, float gamma, int temp) {
	using namespace cv;

	Mat lutRed(1, 256, CV_8U);
	Mat lutGreen(1, 256, CV_8U);
	Mat lutBlue(1, 256, CV_8U);

	// TODO:(Nick) Cache these lookup tables if used every frame...
    uchar* pr = lutRed.ptr();
	uchar* pg = lutGreen.ptr();
	uchar* pb = lutBlue.ptr();
    for( int i = 0; i < 256; ++i) {
        float g = pow(i / 255.0f, gamma) * 255.0f;
		pr[i] = saturate_cast<uchar>(g * kelvinFactor<2>(temp));
		pg[i] = saturate_cast<uchar>(g * kelvinFactor<1>(temp));
		pb[i] = saturate_cast<uchar>(g * kelvinFactor<0>(temp));
	}

	vector<Mat> channels(3);
	split(img, channels);
    LUT(channels[0], lutBlue, channels[0]);
	LUT(channels[1], lutGreen, channels[1]);
	LUT(channels[2], lutRed, channels[2]);
	merge(channels, img);
} 
