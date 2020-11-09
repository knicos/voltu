/**
 * @file voltu.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

// Bump these for each release
#define VOLTU_VERSION_MAJOR 0    // For API incompatible changes
#define VOLTU_VERSION_MINOR 3    // For binary compatibility and extensions
#define VOLTU_VERSION_PATCH 0    // Binary compatible internal fixes

#define VOLTU_VERSION ((VOLTU_VERSION_MAJOR*10000) + (VOLTU_VERSION_MINOR*100) + VOLTU_VERSION_PATCH)

#include <voltu/system.hpp>
#include <voltu/initialise.hpp>
