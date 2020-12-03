/**
 * @file generate.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/codecs/encoder.hpp>
#include <ftl/codecs/opencv_encoder.hpp>
#include <ftl/configurable.hpp>

#include <ftl/config.h>
#include <loguru.hpp>

#include <ftl/codecs/nvidia_encoder.hpp>

namespace ftl {
namespace codecs {
namespace internal {

extern std::list<Encoder*> encoders;
extern bool has_been_init;

void fin_encoders() {
    LOG(INFO) << "Destroying encoders...";
    for (auto *s : encoders) delete s;
}

/* Encoders are a limited resource so precreate a limited number of them. */
void init_encoders() {
	// Nvidia GeForce drivers are limited to 2
    encoders.push_back(new ftl::codecs::NvidiaEncoder);
    encoders.push_back(new ftl::codecs::NvidiaEncoder);

    encoders.push_back(new ftl::codecs::OpenCVEncoder);
    encoders.push_back(new ftl::codecs::OpenCVEncoder);
	encoders.push_back(new ftl::codecs::OpenCVEncoder);
	encoders.push_back(new ftl::codecs::OpenCVEncoder);
	encoders.push_back(new ftl::codecs::OpenCVEncoder);
    encoders.push_back(new ftl::codecs::OpenCVEncoder);
    encoders.push_back(new ftl::codecs::OpenCVEncoder);
	encoders.push_back(new ftl::codecs::OpenCVEncoder);
    encoders.push_back(new ftl::codecs::OpenCVEncoder);

    has_been_init = true;

    atexit(fin_encoders);
}

}
}
}