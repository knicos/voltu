#include "ilw.hpp"

using ftl::ILW;
using ftl::detail::ILWData;

ILW::ILW() {

}

ILW::~ILW() {

}

bool ILW::process(ftl::rgbd::FrameSet &fs) {
    _phase0(fs);

    for (int i=0; i<2; ++i) {
        _phase1(fs);
        for (int j=0; j<3; ++j) {
            _phase2(fs);
        }
    }

    return true;
}

bool ILW::_phase0(ftl::rgbd::FrameSet &fs) {
    // Clear points channel...
}

bool ILW::_phase1(ftl::rgbd::FrameSet &fs) {
    // Run correspondence kernel to find points
}

bool ILW::_phase2(ftl::rgbd::FrameSet &fs) {
    // Run energies and motion kernel
}
