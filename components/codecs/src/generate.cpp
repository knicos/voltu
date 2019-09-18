#include <ftl/codecs/encoder.hpp>
#include <ftl/codecs/opencv_encoder.hpp>
#include <ftl/configurable.hpp>

#include <ftl/config.h>
#include <loguru.hpp>

#ifdef HAVE_NVPIPE
#include <ftl/codecs/nvpipe_encoder.hpp>
#endif

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
    #ifdef HAVE_NVPIPE
    LOG(INFO) << "Adding NvPipe Encoders";
    encoders.push_back(new ftl::codecs::NvPipeEncoder(definition_t::UHD4k, definition_t::HD720));
    encoders.push_back(new ftl::codecs::NvPipeEncoder(definition_t::UHD4k, definition_t::HD720));
    #endif

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