#pragma once

#include "defines.hpp"

#include <voltu/types/frame.hpp>
#include <voltu/types/property.hpp>
#include <memory>
#include <Eigen/Eigen>

namespace voltu
{

enum class ObserverProperty
{
	kInvalid			= 0,
	kBackgroundColour	= 1001,  // String or Int
	kPointCloudMode		= 1002,  // Bool
	kProjection			= 1003,  // Enum or Int
	kAntiAliasing		= 1004,  // Bool
	kDepthOnly			= 1005,  // Bool
	kColourSources		= 1006,  // Bool
	kName				= 1007,  // String
};

class Observer
{
public:
	virtual ~Observer() = default;
	
	PY_API virtual void setResolution(uint32_t w, uint32_t h) = 0;

	PY_API virtual void setFocalLength(uint32_t f) = 0;

	PY_API virtual void setStereo(bool) = 0;

	PY_API virtual bool waitCompletion(int timeout) = 0;

	PY_API virtual void submit(const voltu::FramePtr&) = 0;

	PY_API virtual void setPose(const Eigen::Matrix4f &) = 0;

	PY_API PY_RV_LIFETIME_PARENT virtual voltu::FramePtr getFrame() = 0;

	PY_API virtual voltu::PropertyPtr property(voltu::ObserverProperty) = 0;

	//PY_API virtual voltu::PropertyPtr property(const std::string &name) = 0;
};

typedef std::shared_ptr<Observer> ObserverPtr;

}
