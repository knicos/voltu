#include "property_impl.hpp"
#include <voltu/types/errors.hpp>

using voltu::internal::CfgPropertyImpl;
//using voltu::internal::FloatCfgProperty;

CfgPropertyImpl::CfgPropertyImpl(ftl::Configurable *cfg, const std::string &name)
 : cfg_(cfg), name_(name)
{

}

CfgPropertyImpl::~CfgPropertyImpl()
{

}

void CfgPropertyImpl::setInt(int value)
{
	if (cfg_->is<int>(name_))
	{
		cfg_->set(name_, value);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

void CfgPropertyImpl::setFloat(float value)
{
	if (cfg_->is<float>(name_))
	{
		cfg_->set(name_, value);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

void CfgPropertyImpl::setString(const std::string &value)
{
	if (cfg_->is<std::string>(name_))
	{
		cfg_->set(name_, value);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

void CfgPropertyImpl::setBool(bool value)
{
	if (cfg_->is<bool>(name_))
	{
		cfg_->set(name_, value);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

int CfgPropertyImpl::getInt()
{
	if (cfg_->is<int>(name_))
	{
		return *cfg_->get<int>(name_);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

float CfgPropertyImpl::getFloat()
{
	if (cfg_->is<float>(name_))
	{
		return *cfg_->get<float>(name_);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

std::string CfgPropertyImpl::getString()
{
	if (cfg_->is<std::string>(name_))
	{
		return *cfg_->get<std::string>(name_);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

bool CfgPropertyImpl::getBool()
{
	if (cfg_->is<bool>(name_))
	{
		return *cfg_->get<bool>(name_);
	}
	else
	{
		throw voltu::exceptions::BadPropertyType();
	}
}

// ==== Float ====

/*FloatCfgProperty::FloatCfgProperty(ftl::Configurable *cfg, const std::string &name)
 : CfgPropertyImpl(cfg, name)
{

}

FloatCfgProperty::~FloatCfgProperty()
{

}

void FloatCfgProperty::setFloat(float)
{

}

float FloatCfgProperty::getFloat()
{

}*/
