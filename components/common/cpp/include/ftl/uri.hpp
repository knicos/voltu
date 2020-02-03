#ifndef _FTL_URI_HPP_
#define _FTL_URI_HPP_

#include <nlohmann/json_fwd.hpp>
#include <uriparser/Uri.h>
#include <string>
#include <vector>
#include <map>

namespace ftl {

	typedef const char * uri_t;

	/**
	 * Universal Resource Identifier. Parse, modify, represent and generate URIs.
	 */
	class URI {
		public:
		explicit URI(uri_t puri);
		explicit URI(const std::string &puri);
		explicit URI(const URI &c);

		~URI() {};

		enum scheme_t : int {
			SCHEME_NONE,
			SCHEME_TCP,
			SCHEME_UDP,
			SCHEME_FTL,		// Future Tech Lab
			SCHEME_HTTP,
			SCHEME_WS,
			SCHEME_IPC,
			SCHEME_FILE,
			SCHEME_OTHER,
			SCHEME_DEVICE
		};

		bool isValid() const { return m_valid; };
		const std::string &getHost() const { return m_host; };
		int getPort() const { return m_port; };
		scheme_t getProtocol() const { return m_proto; };
		scheme_t getScheme() const { return m_proto; };
		const std::string &getPath() const { return m_path; };
		const std::string &getFragment() const { return m_frag; }
		std::string getQuery() const;
		const std::string &getBaseURI() const { return m_base; };

		/**
		 * Get the URI without query parameters, and limit path to length N.
		 * If N is negative then it is taken from full path length.
		 */
		std::string getBaseURI(int n);

		std::string getPathSegment(int n) const;

		void setAttribute(const std::string &key, const std::string &value);
		void setAttribute(const std::string &key, int value);

		template <typename T>
		T getAttribute(const std::string &key) {
			return T(m_qmap[key]);
		}

		std::string to_string() const;

		void to_json(nlohmann::json &);

		private:
		void _parse(uri_t puri);

		private:
		bool m_valid;
		std::string m_host;
		std::string m_path;
		std::string m_frag;
		std::string m_base;
		std::vector<std::string> m_pathseg;
		int m_port;
		scheme_t m_proto;
		std::string m_protostr;
		// std::string m_query;
		std::map<std::string, std::string> m_qmap;
	};

	template <>
	inline int URI::getAttribute<int>(const std::string &key) {
		return std::stoi(m_qmap[key]);
	}

	template <>
	inline std::string URI::getAttribute<std::string>(const std::string &key) {
		return m_qmap[key];
	}
}

#endif // _FTL_URI_HPP_
