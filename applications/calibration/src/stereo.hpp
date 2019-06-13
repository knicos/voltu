#ifndef _FTL_CALIBRATION_STEREO_HPP_
#define _FTL_CALIBRATION_STEREO_HPP_

#include <map>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace ftl {
namespace calibration {

void stereo(std::map<std::string, std::string> &opt);

}
}

#endif  // _FTL_CALIBRATION_STEREO_HPP_
