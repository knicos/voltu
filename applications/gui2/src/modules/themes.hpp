#pragma once

#include "../module.hpp"

namespace ftl
{
namespace gui2
{

class Themes : public Module {
public:
	using Module::Module;
	virtual void init() override;
};
}
}
