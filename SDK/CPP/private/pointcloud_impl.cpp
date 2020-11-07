#include "pointcloud_impl.hpp"

using voltu::internal::PointCloudUnstructured;
using voltu::PointCloudData;

PointCloudData PointCloudUnstructured::getHost()
{
	// TODO: Generate point cloud on GPU
	return {};
}

PointCloudData PointCloudUnstructured::getDevice()
{
	// TODO: Generate point cloud on GPU
	return {};
}
