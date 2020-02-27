#include <ftl/exception.hpp>

#ifndef WIN32
#include <execinfo.h>
#endif

using ftl::exception;
using std::string;

exception::exception(const char *msg) : msg_(msg) {
    #ifndef WIN32
    void *trace[16];
    char **messages = (char **)NULL;
    int trace_size = 0;

    trace_size = backtrace(trace, 16);

    trace_ = "[bt] Trace:\n";

    messages = backtrace_symbols(trace, trace_size);

    /* skip first stack frame (points here) */
    for (int i=1; i<trace_size; ++i) {
        printf("[bt] #%d %s\n", i, messages[i]);
        trace_ += string("[bt] #") + std::to_string(i) + string(" ") + messages[i];
    }
    #endif
}

exception::exception(const ftl::Formatter &msg) : msg_(msg.str()) {

}