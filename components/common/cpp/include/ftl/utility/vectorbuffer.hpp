#ifndef _FTL_UTILITY_VECTORBUFFER_HPP_
#define _FTL_UTILITY_VECTORBUFFER_HPP_

#include <vector>

namespace ftl {
namespace util {
class FTLVectorBuffer {
	public:
	inline explicit FTLVectorBuffer(std::vector<unsigned char> &v) : vector_(v) {}

	inline void write(const char *data, std::size_t size) {
		vector_.insert(vector_.end(), (const unsigned char*)data, (const unsigned char*)data+size);
	}

	private:
	std::vector<unsigned char> &vector_;
};
}
}

#endif  // _FTL_UTILITY_VECTORBUFFER_HPP_
