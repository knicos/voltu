#ifndef _FTL_URI_HPP_
#define _FTL_URI_HPP_

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
			SCHEME_OTHER
		};

		bool isValid() const { return m_valid; };
		const std::string &getHost() const { return m_host; };
		int getPort() const { return m_port; };
		scheme_t getProtocol() const { return m_proto; };
		scheme_t getScheme() const { return m_proto; };
		const std::string &getPath() const { return m_path; };
		std::string getQuery() const;
		const std::string &getBaseURI() const { return m_base; };
		const std::string &getPathSegment(int n) const { return m_pathseg[n]; };

		void setAttribute(const std::string &key, const std::string &value);
		void setAttribute(const std::string &key, int value);

		template <typename T>
		T getAttribute(const std::string &key) {
			return T(m_qmap[key]);
		}

		std::string to_string() const;

		private:
		void _parse(uri_t puri);

		private:
		bool m_valid;
		std::string m_host;
		std::string m_path;
		std::string m_base;
		std::vector<std::string> m_pathseg;
		int m_port;
		scheme_t m_proto;
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
