#pragma once

#include <voltu/types/property.hpp>
#include <memory>

namespace voltu
{

enum class OperatorId
{
	kInvalid		= 0,
	kDepth			= 1,
	kGTEvaluator	= 2,
	kFusion			= 3,
	kClipping		= 4,
	kAruco			= 5,
	kPixelAnalysis	= 6
};

class Operator
{
public:
	virtual ~Operator() = default;
	
	PY_API virtual voltu::PropertyPtr property(const std::string &name) = 0;
};

typedef std::shared_ptr<Operator> OperatorPtr;

}