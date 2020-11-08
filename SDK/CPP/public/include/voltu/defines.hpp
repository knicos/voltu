/**
 * @file system.hpp
 * @copyright Copyright (c) 2020 Sebastian Hahta, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#ifndef PY_API
/// include function or method in Python API
#define PY_API
#endif

#ifndef PY_NO_SHARED_PTR
/// Ownership is not passed with std::shared_ptr<>
#define PY_NO_SHARED_PTR
#endif

#ifndef PY_RV_LIFETIME_PARENT
/// Lifetime of the return value is tied to the lifetime of a parent object
#define PY_RV_LIFETIME_PARENT
#endif
