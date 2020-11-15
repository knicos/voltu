#pragma once

#include <voltu/pipeline.hpp>
#include <ftl/operators/operator.hpp>

namespace voltu
{
namespace internal
{

class PipelineImpl : public voltu::Pipeline
{
public:
	PipelineImpl(ftl::Configurable *root);
	~PipelineImpl() override;

	void submit(const voltu::FramePtr &frame) override;

	bool waitCompletion(int timeout, bool except) override;

	voltu::OperatorPtr appendOperator(voltu::OperatorId id) override;

private:
	ftl::operators::Graph *graph_;
	bool ready_ = false;
};

}
}