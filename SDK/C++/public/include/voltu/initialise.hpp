#pragma once
#include <memory>
#include <voltu/system.hpp>
#include "defines.hpp"

namespace voltu
{
	PY_API std::shared_ptr<voltu::System> instance();
}
