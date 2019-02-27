#ifndef _FTL_UUID_HPP_
#define _FTL_UUID_HPP_

#include <uuid/uuid.h>
#include <memory>
#include <string>
#include <functional>
#include <msgpack.hpp>

namespace ftl {
	class UUID {
		public:
		UUID() { uuid_generate(uuid_); };
		UUID(const UUID &u) { memcpy(uuid_,u.uuid_,16); }
		
		bool operator==(const UUID &u) const { return memcmp(uuid_,u.uuid_,16) == 0; }
		bool operator!=(const UUID &u) const { return memcmp(uuid_,u.uuid_,16) != 0; }
		
		std::string str() const { return std::string((char*)uuid_,16); };
		const unsigned char *raw() const { return &uuid_[0]; }
		
		MSGPACK_DEFINE(uuid_);
		
		private:
		unsigned char uuid_[16];
	};
};

namespace std {
	template <> struct hash<ftl::UUID> {
		size_t operator()(const ftl::UUID & x) const {
			return std::hash<std::string>{}(x.str());
		}
	};
};

#endif // _FTL_UUID_HPP_

