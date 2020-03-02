#include <ftl/exception.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#ifndef WIN32
#include <execinfo.h>
#endif

using ftl::exception;
using std::string;

static std::string getBackTrace() {
#ifndef WIN32

	string result;
    void *trace[16];
    int trace_size = 0;

    trace_size = backtrace(trace, 16);

    result = "[bt] Trace:\n";

    char **messages = backtrace_symbols(trace, trace_size);

    /* skip first stack frame (points here) */
    for (int i=2; i<trace_size; ++i) {
        //printf("[bt] #%d %s\n", i, messages[i]);
        result += string("[bt] #") + std::to_string(i-1) + string(" ") + messages[i] + string("\n");
    }
	return result;

#else
	return "";
#endif
}

exception::exception(const char *msg) : msg_(msg), processed_(false) {
   trace_ = std::move(getBackTrace());
}

exception::exception(const ftl::Formatter &msg) : msg_(msg.str()), processed_(false) {
	trace_ = std::move(getBackTrace());
}

exception::~exception() {
	if (!processed_) {
		LOG(ERROR) << "Unreported exception: " << what();
		LOG(ERROR) << trace_;
	}
}