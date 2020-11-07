/**
 * @file operator.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include <voltu/types/property.hpp>
#include <memory>

namespace voltu
{


/**
 * @brief Operation type identifier.
 */
enum class OperatorId
{
	kInvalid		= 0,
	kDepth			= 1,	///< Calculate depth map from stereo images
	kGTEvaluator	= 2,	///< Ground truth quality evaluation
	kFusion			= 3,	///< Fuse multiple depth maps together
	kClipping		= 4,	///< Use a clip box to remove unwanted data
	kAruco			= 5,	///< Aruco tag detector
	kPixelAnalysis	= 6		// TODO: Change this
};

/**
 * @brief Manage operator settings.
 */
class Operator
{
public:
	virtual ~Operator() = default;
	
	/**
	 * @brief Get a named operator property.
	 * 
	 * @throw voltu::exceptions::BadPropertyName If name not valid.
	 * @return Property accessor object.
	 */
	PY_API virtual voltu::PropertyPtr property(const std::string &name) = 0;

	// TODO: Get list of properties supported
};

typedef std::shared_ptr<Operator> OperatorPtr;

}