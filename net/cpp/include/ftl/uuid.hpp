#ifndef _FTL_UUID_HPP_
#define _FTL_UUID_HPP_

#ifndef WIN32
#include <uuid/uuid.h>
#else
#include <Rpc.h>
#endif

#include <memory>
#include <string>
#include <functional>
#include <msgpack.hpp>

namespace ftl {
	/**
	 * C++ Wrapper for libuuid. The default constructor generates a new UUID.
	 */
	class UUID {
		public:
		UUID() {
#ifdef WIN32
			::UuidCreate(&uuid_);
#else
			uuid_generate(uuid_);
#endif
		};
		UUID(int u) { memset(&uuid_,u,16); };
		UUID(const UUID &u) { memcpy(&uuid_,&u.uuid_,16); }
		
		bool operator==(const UUID &u) const { return memcmp(&uuid_,&u.uuid_,16) == 0; }
		bool operator!=(const UUID &u) const { return memcmp(&uuid_,&u.uuid_,16) != 0; }
		
		/**
		 * Get a raw data string.
		 */
		std::string str() const { return std::string((char*)&uuid_,16); };
		const unsigned char *raw() const { return (const unsigned char*)&uuid_; }
		
		/**
		 * Get a pretty string.
		 */
		std::string to_string() const {
			char b[37];
#ifdef WIN32
			UuidToString(&uuid_, (RPC_CSTR*)b);
#else
			uuid_unparse(uuid_, b);
#endif
			return std::string(b);
		}
		
		/* Allow the UUID to be packed into an RPC message. */
		MSGPACK_DEFINE(uuid_);
		
		private:
#ifdef WIN32
		_GUID uuid_;
#else
		unsigned char uuid_[16];
#endif
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

