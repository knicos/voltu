#pragma once

#include <memory>

namespace voltu
{

enum class PointCloudFormat
{
	kInvalid = 0,
	kUnstructured = 1
};

enum class PointFormat
{
	kInvalid = 0,
	kXYZ_Float = 1,
	kXYZRGB_Float = 2,
	kXYZ_NxNyNz_Float = 3,
	kXYZ_NxNyNz_RGB_Float = 4,
	kXYZ_Float_RGB_Int8 = 5,
	kXYZ_NxNyNz_Float_RGB_Int8 = 6
};

struct PointCloudData
{
	PointCloudFormat format = PointCloudFormat::kInvalid;
	unsigned char* data = nullptr;
	unsigned int element_size;
	size_t size;
};

class PointCloud
{
public:
	virtual PointCloudData getHost() = 0;

	virtual PointCloudData getDevice() = 0;
};

typedef std::shared_ptr<PointCloud> PointCloudPtr;

}
