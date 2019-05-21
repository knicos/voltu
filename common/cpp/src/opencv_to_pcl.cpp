#include <ftl/utility/opencv_to_pcl.hpp>

#if defined HAVE_PCL
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ftl::utility::matToPointXYZ(const cv::Mat &cvcloud, const cv::Mat &rgbimage) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = cvcloud.cols * cvcloud.rows;
	point_cloud_ptr->height = 1;

	for(int i=0;i<cvcloud.rows;i++) {
	for(int j=0;j<cvcloud.cols;j++) {
		//std::cout<<i<<endl;

		pcl::PointXYZRGB point;
		cv::Vec3f cvpoint = cvcloud.at<cv::Vec3f>(i,j);
		point.x = cvpoint[0];
		point.y = cvpoint[1];
		point.z = cvpoint[2];

		if (point.x == INFINITY || point.y == INFINITY || point.z == INFINITY || point.z < 600.0f) {
			point.x = 0; point.y = 0; point.z = 0;
		}

		cv::Point3_<uchar> prgb = rgbimage.at<cv::Point3_<uchar>>(i, j);

		// when color needs to be added:
		uint32_t rgb = (static_cast<uint32_t>(prgb.z) << 16 | static_cast<uint32_t>(prgb.y) << 8 | static_cast<uint32_t>(prgb.x));
		point.rgb = *reinterpret_cast<float*>(&rgb);

		point_cloud_ptr -> points.push_back(point);
	}
	}

	return point_cloud_ptr;
}
#endif  // HAVE_PCL