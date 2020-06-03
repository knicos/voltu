#include <ftl/exception.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#ifdef __GNUC__
#include <execinfo.h>
#include <dlfcn.h>
#include <cxxabi.h>
#endif

using ftl::exception;
using std::string;

static std::string demangle(const char* name) {
	if (!name) {
		return "[unknown symbol]";
	}
#ifdef __GNUC__
	int status;
	char* demangled = abi::__cxa_demangle(name, NULL, 0, &status);
	if (!demangled) {
		return std::string(name);
	}
	else {
		auto result = string(demangled);
		free(demangled);
		return result;
	}
#else
	return std::string(name);
#endif
}

static std::string addr_to_string(const void* addr) {
	std::stringstream ss;
	ss << addr;
	return ss.str();
}

static std::string getBackTrace() {
#ifdef __GNUC__
	string result;
	void *trace[16];
	int trace_size = 0;
	trace_size = backtrace(trace, 16);
	char **messages = backtrace_symbols(trace, trace_size);

	result = "[bt] Trace:\n";

	/* skip first stack frame (points here) */
	for (int i=2; i < trace_size; ++i) {
		result += string("[bt] #") + std::to_string(i-1) + string(" ");

		Dl_info info;
		if (dladdr(trace[i], &info) && info.dli_saddr) {
			auto name = demangle(info.dli_sname);
			string fname = info.dli_fname ? info.dli_fname: "[unknown file]";

			result += fname +
				+ "           "
				+ " [" + addr_to_string(info.dli_saddr) + "]" // exact address of symbol
				+ string(", in ")
				+ name;
		}
		else {
			result += messages[i];
		}
		result += "\n";
	}

	free(messages);
	return result;

#elif _MSC_VER
	return "";
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
		LOG(ERROR) << "Unhandled exception: " << what();
		LOG(ERROR) << trace_;
	}
}
