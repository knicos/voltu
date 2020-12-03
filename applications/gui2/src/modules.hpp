/**
 * @file modules.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include "modules/thumbnails.hpp"
#include "modules/camera.hpp"
#include "modules/config.hpp"
#include "modules/themes.hpp"
#include "modules/statistics.hpp"
#ifdef HAVE_CERES
#include "modules/calibration/calibration.hpp"
#endif
#include "modules/addsource.hpp"
#include "modules/dev/developer.hpp"
