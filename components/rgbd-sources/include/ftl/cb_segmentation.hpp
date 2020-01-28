#pragma once

#include <opencv2/core.hpp>

namespace ftl {

/**
 * @brief	Codebook segmentation and depthmap filling.
 * @param	Input image width
 * @param	Input image height
 * 
 * Codebook segmentation based on
 *
 * Kim, K., Chalidabhongse, T. H., Harwood, D., & Davis, L. (2005).
 * Real-time foreground-background segmentation using codebook model.
 * Real-Time Imaging. https://doi.org/10.1016/j.rti.2004.12.004
 * 
 * and fixed size codebook optimization in
 * 
 * Rodriguez-Gomez, R., Fernandez-Sanchez, E. J., Diaz, J., & Ros, E.
 * (2015). Codebook hardware implementation on FPGA for background
 * subtraction. Journal of Real-Time Image Processing.
 * https://doi.org/10.1007/s11554-012-0249-6
 * 
 * Additional modifications to include depth maps as part of the
 * background model.
 */
class CBSegmentation {
public:
	CBSegmentation(char codebook_size, size_t width, size_t height, float alpha, float beta, float epsilon, float sigma, int T_add, int T_del, int T_h);

	/**
	 * @brief	Segment image.
	 * @param	Input image (3-channels)
	 * @param	Output Mat. Background pixels set to 0, foreground pixels > 0.
	 *
	 * @todo	Template method on OpenCV type
	 */
	void apply(cv::Mat &in, cv::Mat &out, cv::Mat &depth, bool fill=false);
	void apply(cv::Mat &in, cv::Mat &out);
	
protected:
	class Pixel {
	public:
		int idx;
		float r;
		float g;
		float b;
		float i;
		int d;
		long t;
		Pixel(const int &index, const uchar *bgr, const int &depth, const long &time);
	};

	class Codeword {
	public:
		float r;
		float g;
		float b;
		float i_min, i_max;
		long f, lambda, p, q;

		float d_m;
		float d_f;
		float d_S;
		
		void set(CBSegmentation::Pixel &pixel);
		void update(CBSegmentation::Pixel &pixel);

		bool colordiff(CBSegmentation::Pixel &pixel, float epsilon);
		bool brightness(CBSegmentation::Pixel &pixel, float alpha, float beta);
		bool depthdiff(CBSegmentation::Pixel &pixel, float sigma);

		inline int freq() { return f; }
		inline long getLambda() { return lambda; }
		inline long ctime() { return p; }
		inline long atime() { return q; }
	};

	enum EntryType { H, M };

	union Entry {
		char size;
		struct Data {
			EntryType type;
			CBSegmentation::Codeword cw;
		} data ;
	};

	struct CompareEntry{
		bool operator()(const Entry &a,const Entry &b) const{
			return 	!((a.data.type == M && b.data.type == H) ||
					(a.data.cw.f < b.data.cw.f));
		}
	};

	bool processPixel(Pixel &px, Codeword *codeword=nullptr);
	
	size_t size_;
	size_t width_;
	size_t height_;

	float alpha_;
	float beta_;
	float epsilon_;
	float sigma_;

	int T_add_;
	int T_del_;
	int T_h_;

private:
	long t_ = 1;
	std::vector<Entry> cb_;
};

}