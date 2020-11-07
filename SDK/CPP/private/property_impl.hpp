#pragma once

#include <ftl/configurable.hpp>
#include <voltu/types/property.hpp>
#include <string>

namespace voltu
{
namespace internal
{

class CfgPropertyImpl : public voltu::Property
{
public:
	CfgPropertyImpl(ftl::Configurable *cfg, const std::string &name);
	virtual ~CfgPropertyImpl() override;

	virtual void setInt(int) override;

	virtual void setFloat(float) override;

	virtual void setString(const std::string &) override;

	virtual void setBool(bool) override;

	virtual int getInt() override;

	virtual float getFloat() override;

	virtual std::string getString() override;

	virtual bool getBool() override;

private:
	ftl::Configurable *cfg_;
	std::string name_;
};

/*class FloatCfgProperty : public CfgPropertyImpl
{
public:
	FloatCfgProperty(ftl::Configurable *cfg, const std::string &name);
	~FloatCfgProperty() override;

	void setFloat(float) override;

	float getFloat() override;
};*/

}
}
