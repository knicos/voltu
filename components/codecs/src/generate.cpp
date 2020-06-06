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

void init_encoders() {
    encoders.push_back(new ftl::codecs::NvidiaEncoder(definition_t::UHD4k, definition_t::HD720));
    encoders.push_back(new ftl::codecs::NvidiaEncoder(definition_t::UHD4k, definition_t::HD720));

    encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::HD1080, definition_t::HD720));
    encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::HD1080, definition_t::HD720));
	encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::HD1080, definition_t::HD720));
	encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::HD1080, definition_t::HD720));
	encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::HD1080, definition_t::HD720));
    encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::SD576, definition_t::LD360));
    encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::SD576, definition_t::LD360));
	encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::SD576, definition_t::LD360));
    encoders.push_back(new ftl::codecs::OpenCVEncoder(definition_t::SD576, definition_t::LD360));

    has_been_init = true;

    atexit(fin_encoders);
}

}
}
}