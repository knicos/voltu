/**
 * @file pipeline.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include <voltu/defines.hpp>
#include <voltu/types/frame.hpp>
#include <voltu/operator.hpp>
#include <memory>

namespace voltu
{

class Pipeline
{
public:
	virtual ~Pipeline() = default;
	
	PY_API virtual void submit(const voltu::FramePtr &frame) = 0;

	PY_API virtual bool waitCompletion(int timeout) = 0;

	PY_API virtual voltu::OperatorPtr appendOperator(voltu::OperatorId id) = 0;
};

typedef std::shared_ptr<Pipeline> PipelinePtr;

}
