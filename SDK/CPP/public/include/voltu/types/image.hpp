/**
 * @file image.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include "../defines.hpp"

#include <voltu/types/channel.hpp>
#include <voltu/types/intrinsics.hpp>
#include <memory>

#include <Eigen/Eigen>

namespace voltu
{

enum class ImageFormat
{
	kInvalid = 0,
	kFloat32 = 1,
	kBGRA8 = 2,
	kFloat16_4 = 3
};

PY_NO_SHARED_PTR struct ImageData
{
	ImageFormat format = ImageFormat::kInvalid;
	unsigned char* data = nullptr;
	unsigned int pitch = 0;
	unsigned int width = 0;
	unsigned int height = 0;
};

class Image
{
public:
	virtual ~Image() = default;

	PY_API PY_RV_LIFETIME_PARENT virtual ImageData getHost() = 0;

	virtual ImageData getDevice() = 0;

	virtual bool isDevice() = 0;

	PY_API virtual voltu::Channel getChannel() = 0;

	PY_API virtual std::string getName() = 0;

	PY_API virtual voltu::Intrinsics getIntrinsics() = 0;

	PY_API virtual Eigen::Matrix4d getPose() = 0;

	PY_API virtual voltu::StereoIntrinsics getStereoIntrinsics() = 0;

	PY_API virtual int64_t getTimestamp() = 0;

	//virtual voltu::RoomId getRoomId() = 0;

	PY_API virtual int getCameraNumber() = 0;

	PY_API virtual uint32_t getUniqueId() = 0;
};

typedef std::shared_ptr<Image> ImagePtr;

}
