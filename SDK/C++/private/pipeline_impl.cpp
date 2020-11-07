#include "pipeline_impl.hpp"
#include "frame_impl.hpp"
#include "operator_impl.hpp"
#include <voltu/types/errors.hpp>

#include <ftl/operators/fusion.hpp>
#include <ftl/operators/gt_analysis.hpp>

using voltu::internal::PipelineImpl;

PipelineImpl::PipelineImpl(ftl::Configurable *root)
{
	graph_ = ftl::create<ftl::operators::Graph>(root, "pipe1");
}

PipelineImpl::~PipelineImpl()
{
	delete graph_;
}

void PipelineImpl::submit(const voltu::FramePtr &frame)
{
	auto *fimp = dynamic_cast<voltu::internal::FrameImpl*>(frame.get());
	if (!fimp)
	{
		throw voltu::exceptions::InvalidFrameObject();
	}

	const auto &sets = fimp->getInternalFrameSets();

	if (sets.size() > 1) throw voltu::exceptions::IncompatibleOperation();

	for (const auto &fs : sets)
	{
		ready_ = false;
		graph_->queue(fs, [this]()
		{
			ready_ = true;
		});
	}
}

bool PipelineImpl::waitCompletion(int timeout)
{
	int count = timeout / 5;
	while (!ready_ && --count >= 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
	return ready_;
}

voltu::OperatorPtr PipelineImpl::appendOperator(voltu::OperatorId id)
{
	switch (id)
	{
	case voltu::OperatorId::kFusion			: return std::make_shared<voltu::internal::OperatorImpl>(graph_->append<ftl::operators::Fusion>("fusion"));
	case voltu::OperatorId::kGTEvaluator	: return std::make_shared<voltu::internal::OperatorImpl>(graph_->append<ftl::operators::GTAnalysis>("gtanal"));
	default: throw voltu::exceptions::NotImplemented();
	}
}