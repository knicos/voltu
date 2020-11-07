#pragma once

#include <voltu/operator.hpp>
#include <ftl/configurable.hpp>

#include <string>

namespace voltu
{
namespace internal
{

class OperatorImpl : public voltu::Operator
{
public:
	OperatorImpl(ftl::Configurable*);
	~OperatorImpl() override;

	voltu::PropertyPtr property(const std::string &name) override;

private:
	ftl::Configurable *cfg_;
};

}
}