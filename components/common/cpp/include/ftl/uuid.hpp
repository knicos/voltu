/**
 * @file uuid.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

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

#include <ftl/utility/msgpack.hpp>

namespace ftl {
	/**
	 * C++ Wrapper for libuuid. The default constructor generates a new UUID.
	 */
	class UUID {
		public:
		UUID() {
#ifdef WIN32
			::UuidCreate(&guid_);
#else
			uuid_generate(uuid_);
#endif
		}
		explicit UUID(int u) { memset(uuid_,u,16); }
		UUID(const ftl::UUID &u) { memcpy(uuid_,u.uuid_,16); }
		explicit UUID(const std::string &s) {
#ifdef WIN32
			// TODO(Nick) Windows UUID parse
#else
			if (uuid_parse(s.c_str(), uuid_) < 0) {
				memset(uuid_,0,16);
			}
#endif
		}
		
		UUID &operator=(const UUID &u)  {
			memcpy(&uuid_,&u.uuid_,16); return *this;
		}
		bool operator==(const UUID &u) const {
			return memcmp(&uuid_,&u.uuid_,16) == 0;
		}
		bool operator!=(const UUID &u) const {
			return memcmp(&uuid_,&u.uuid_,16) != 0;
		}
		bool operator<(const UUID &u) const {
			return strncmp((const char*)uuid_, (const char *)u.uuid_, 16) < 0;
		}
		
		/**
		 * Get a raw data string.
		 */
		std::string str() const { return std::string((char*)&uuid_,16); }
		const unsigned char *raw() const { return (const unsigned char*)&uuid_; }
		
		/**
		 * Get a pretty string.
		 */
		std::string to_string() const {
			static const char *digits = "0123456789abcdef";
			std::string rc(sizeof(uuid_)*2+4,'0');

			size_t j=0;
			for (size_t i=0 ; i<4; ++i) {
				rc[j+1] = digits[uuid_[i] & 0x0f];
				rc[j] = digits[(uuid_[i] >> 4) & 0x0f];
				j+=2;
			}
			rc[j++] = '-';
			for (size_t i=4 ; i<6; ++i) {
				rc[j+1] = digits[uuid_[i] & 0x0f];
				rc[j] = digits[(uuid_[i] >> 4) & 0x0f];
				j+=2;
			}
			rc[j++] = '-';
			for (size_t i=6 ; i<8; ++i) {
				rc[j+1] = digits[uuid_[i] & 0x0f];
				rc[j] = digits[(uuid_[i] >> 4) & 0x0f];
				j+=2;
			}
			rc[j++] = '-';
			for (size_t i=8 ; i<10; ++i) {
				rc[j+1] = digits[uuid_[i] & 0x0f];
				rc[j] = digits[(uuid_[i] >> 4) & 0x0f];
				j+=2;
			}
			rc[j++] = '-';
			for (size_t i=10 ; i<16; ++i) {
				rc[j+1] = digits[uuid_[i] & 0x0f];
				rc[j] = digits[(uuid_[i] >> 4) & 0x0f];
				j+=2;
			}
			return rc;
/* 
#ifdef WIN32
			RPC_CSTR szUuid = NULL;
			if (::UuidToStringA(&guid_, &szUuid) == RPC_S_OK) {
				return std::string((char*)szUuid);
			}
			return "00000000-0000-0000-0000-000000000000";
#else
			char b[37];
			uuid_unparse(uuid_, b);
			return std::string(b);
#endif
*/
		}
		
		/* Allow the UUID to be packed into an RPC message. */
		MSGPACK_DEFINE(uuid_);
		
		private:
#ifdef WIN32
		union {
			_GUID guid_;
			unsigned char uuid_[16];
		};
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

