#include "operator_impl.hpp"
#include "property_impl.hpp"
#include <voltu/types/errors.hpp>

using voltu::internal::OperatorImpl;

OperatorImpl::OperatorImpl(ftl::Configurable *cfg)
 : cfg_(cfg)
{
	
}

OperatorImpl::~OperatorImpl()
{

}

voltu::PropertyPtr OperatorImpl::property(const std::string &name)
{
	if (!cfg_->has(name)) throw voltu::exceptions::BadPropertyName();
	return std::make_shared<voltu::internal::CfgPropertyImpl>(cfg_, name);
}
