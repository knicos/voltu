#include <ftl/exception.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#ifdef __GNUC__
#include <execinfo.h>
#include <dlfcn.h>
#include <cxxabi.h>

std::string demangle(const char* name) {
	if (!name) {
		return "[unknown symbol]";
	}
	int status;
	char* demangled = abi::__cxa_demangle(name, NULL, 0, &status);
	if (!demangled) {
		return std::string(name);
	}
	else {
		auto result = std::string(demangled);
		free(demangled);
		return result;
	}
}

#endif

using ftl::exception;
using std::string;

string addr_to_string(const void* addr) {
	std::stringstream ss;
	ss << addr;
	return ss.str();
}

#ifdef __GNUC__
string exception::decode_backtrace() const {
	string result;
	// backtrace_symbols() as fallback (no data from dladdr())
	char **messages = backtrace_symbols(trace_, trace_size_);

	if (!messages) {
		return string("[bt] no trace");
	}

	/* skip first stack frame (points here) */
	for (int i=1; i < trace_size_; ++i) {
		result += string("[bt] #") + std::to_string(i-1)
				+ string(TRACE_SIZE_MAX_/10 - (i-1)/10, ' ')
				+ string(" ");

		Dl_info info;
		if (dladdr(trace_[i], &info) && info.dli_saddr) {
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
}

#else
string exception::decode_backtrace() const {
	return string();
}
#endif

exception::exception(const char *msg) : msg_(msg), processed_(false) {
	#ifdef __GNUC__
	trace_size_ = backtrace(trace_, TRACE_SIZE_MAX_);
	#endif
}

exception::exception(const ftl::Formatter &msg) : msg_(msg.str()), processed_(false) {
	#ifdef __GNUC__
	trace_size_ = backtrace(trace_, TRACE_SIZE_MAX_);
	#endif
}

exception::~exception() {
	if (!processed_) {
		LOG(ERROR) << "Unhandled exception: " << what();
		#ifdef __GNUC__
		LOG(ERROR) << "Trace:\n" << decode_backtrace();
		#endif
	}
}
