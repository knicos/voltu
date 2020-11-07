#pragma once

#include <voltu/types/pointcloud.hpp>

namespace voltu
{
namespace internal
{

class PointCloudUnstructured : public voltu::PointCloud
{
public:
	voltu::PointCloudData getHost() override;

	voltu::PointCloudData getDevice() override;

private:
};

}
}
