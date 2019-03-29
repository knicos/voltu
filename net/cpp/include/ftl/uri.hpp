#ifndef _FTL_URI_HPP_
#define _FTL_URI_HPP_

#include <uriparser/Uri.h>
#include <string>
#include <vector>

namespace ftl {

	typedef const char * uri_t;

	class URI {
		public:
		URI(uri_t puri) {
			UriUriA uri;

			#ifdef HAVE_URIPARSESINGLE
			const char *errpos;
			if (uriParseSingleUriA(&uri, puri, &errpos) != URI_SUCCESS) {
			#else
			UriParserStateA uris;
			uris.uri = &uri;
			if (uriParseUriA(&uris, puri) != URI_SUCCESS) {
			#endif
				m_valid = false;
				m_host = "none";
				m_port = -1;
				m_proto = SCHEME_NONE;
				m_path = "";
			} else {
				m_host = std::string(uri.hostText.first, uri.hostText.afterLast - uri.hostText.first);
				
				std::string prototext = std::string(uri.scheme.first, uri.scheme.afterLast - uri.scheme.first);
				if (prototext == "tcp") m_proto = SCHEME_TCP;
				else if (prototext == "udp") m_proto = SCHEME_UDP;
				else if (prototext == "ftl") m_proto = SCHEME_FTL;
				else if (prototext == "http") m_proto = SCHEME_HTTP;
				else if (prototext == "ws") m_proto = SCHEME_WS;
				else if (prototext == "ipc") m_proto = SCHEME_IPC;
				else m_proto = SCHEME_OTHER;

				std::string porttext = std::string(uri.portText.first, uri.portText.afterLast - uri.portText.first);
				m_port = atoi(porttext.c_str());

				for (auto h=uri.pathHead; h!=NULL; h=h->next) {
					auto pstr = std::string(
							h->text.first, h->text.afterLast - h->text.first);

					m_path += "/";
					m_path += pstr;
					m_pathseg.push_back(pstr);
				}

				m_query = std::string(uri.query.first, uri.query.afterLast - uri.query.first);

				uriFreeUriMembersA(&uri);

				m_valid = m_proto != SCHEME_NONE && m_host.size() > 0;

				if (m_valid) {
					if (m_query.size() > 0) m_base = std::string(uri.scheme.first, uri.query.first - uri.scheme.first - 1);
					else m_base = std::string(uri.scheme.first);
				}
			}
		}

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
		const std::string &getQuery() const { return m_query; };
		const std::string &getBaseURI() const { return m_base; };
		const std::string &getPathSegment(int n) const { return m_pathseg[n]; };

		private:
		bool m_valid;
		std::string m_host;
		std::string m_path;
		std::string m_base;
		std::vector<std::string> m_pathseg;
		int m_port;
		scheme_t m_proto;
		std::string m_query;
	};
}

#endif // _FTL_URI_HPP_
