/**
 * @file initialise.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once
#include <memory>
#include <voltu/system.hpp>
#include "defines.hpp"

namespace voltu
{
	PY_API std::shared_ptr<voltu::System> instance();
}
