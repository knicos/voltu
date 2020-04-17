#include <thrust/device_ptr.h>
#include <thrust/fill.h>

template <typename T>
DisparitySpaceImage<T>::DisparitySpaceImage() : DSBase<DisparitySpaceImage<T>::DataType>(0,0,0,0) {
	this->data_.data = nullptr;
	needs_free_ = false;
}

template <typename T>
DisparitySpaceImage<T>::DisparitySpaceImage(int width, int height, int disp_min, int disp_max)
	: DSBase<DisparitySpaceImage<T>::DataType>(width, height, disp_min, disp_max)
{
	this->data_.data = allocateMemory2D<T>(width*(disp_max-disp_min+1), height, this->data_.pitch);
	needs_free_ = true;
}

template <typename T>
DisparitySpaceImage<T>::~DisparitySpaceImage()
{
	if (needs_free_ && this->data_.data) freeMemory(this->data_.data);
}

template <typename T>
void DisparitySpaceImage<T>::create(int w, int h, int dmin, int dmax) {
	if (this->data_.data && this->data_.width == w && this->data_.height == h) return;
	if (needs_free_) freeMemory(this->data_.data);
	needs_free_ = true;
	this->data_.width = w;
	this->data_.height = h;
	this->data_.disp_min = dmin;
	this->data_.disp_max = dmax;
	this->data_.data = allocateMemory2D<T>(w*(dmax-dmin+1), h, this->data_.pitch);
}

template <typename T>
void DisparitySpaceImage<T>::clear() {
#ifdef USE_GPU
	//if (this->width() == 1 || this->height() == 1) {
		cudaSafeCall(cudaMemset(this->data_.data, 0, this->data_.pitch*sizeof(T)*this->data_.height));
	//} else {
	//	cudaSafeCall(cudaMemset2D(this->data_.data, this->data_.pitch*sizeof(T), 0, this->data_.width*this->numDisparities()*sizeof(T), this->data_.height));
	//}
#else
	memset(this->data_.data, 0, this->data_.pitch*sizeof(T)*this->data_.height);
#endif
}

template <typename T>
T DisparitySpaceImage<T>::operator()(int y, int x, int d) const {
#ifdef USE_GPU
	T t;
	cudaMemcpy(&t, &this->data_(y,x,d), sizeof(T), cudaMemcpyDeviceToHost);
	return t;
#else
	return this->data_(y,x,d);
#endif
}

template <typename T>
void DisparitySpaceImage<T>::set(const T& v) {
#ifdef USE_GPU
	thrust::device_ptr<T> dev_ptr(this->data_.data);
	thrust::fill(dev_ptr, dev_ptr + this->data_.size(), v);
#else
	std::fill(this->data_.data, this->data_.data + this->data_.size(), v);
#endif
}

template <typename T>
template<typename A>
void DisparitySpaceImage<T>::add(const A &other, double scale) {
#ifdef USE_GPU

#else
	#pragma omp parallel for
	for (int y = 0; y < this->data_.height; y++)
	for (int x = 0; x < this->data_.width; x++)
	for (int d = this->data_.disp_min; d <= this->data_.disp_max; d++) {{{
		this->data_(y,x,d) += other(y,x,d)*scale;
	}}}
#endif
}

template <typename T>
void DisparitySpaceImage<T>::mul(double a) {
	std::transform(this->data_.begin(), this->data_.end(), this->data_.begin(), [&a](T &elem) { return elem *= a; });
}

template <typename T>
cv::Mat DisparitySpaceImage<T>::Mat() {
	int cvtype;

	if (std::is_same<char, T>::value) {
		cvtype = CV_8SC1;
	}
	else if (std::is_same<short, T>::value) {
		cvtype = CV_16SC1;
	}
	else if (std::is_same<int, T>::value) {
		cvtype = CV_32SC1;
	}
	else if (std::is_same<float, T>::value) {
		cvtype = CV_32FC1;
	}
	else if (std::is_same<double, T>::value) {
		cvtype = CV_64FC1;
	}
	else {
		return cv::Mat();
	}

#ifdef USE_GPU

#else
	return cv::Mat(this->data_.width*this->data_.height*this->data_.disparityRange(), 1, cvtype, this->data_.data);
#endif
}
