/**
 * @file defines.hpp
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

#ifndef PY_SINGLETON
/// Singleton instance, members exported to module. Requires creating the
/// instance in PyModule constructor.
#define PY_SINGLETON
#endif

#ifndef PY_SINGLETON_OBJECT
/// Export as singleton instance instead of exporting members to module
#define PY_SINGLETON_OBJECT
#endif

