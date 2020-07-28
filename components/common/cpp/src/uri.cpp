#include <ftl/uri.hpp>
#include <nlohmann/json.hpp>
// #include <filesystem>  TODO When available
#include <cstdlib>
#include <loguru.hpp>

#ifndef WIN32
#include <unistd.h>
#else
#include <direct.h>
#endif

using ftl::URI;
using ftl::uri_t;
using std::string;

URI::URI(uri_t puri) {
	_parse(puri);
}

URI::URI(const std::string &puri) {
	_parse(puri.c_str());
}

URI::URI(const URI &c) {
	m_valid = c.m_valid;
	m_host = c.m_host;
	m_port = c.m_port;
	m_proto = c.m_proto;
	m_path = c.m_path;
	m_pathseg = c.m_pathseg;
	m_qmap = c.m_qmap;
	m_base = c.m_base;
	m_userinfo = c.m_userinfo;
	m_frag = c.m_frag;
}

void URI::_parse(uri_t puri) {
	UriUriA uri;

	std::string suri = puri;

	// NOTE: Non-standard additions to allow for Unix style relative file names.
	if (suri[0] == '.') {
		char cwdbuf[1024];
		if (getcwd(cwdbuf, 1024)) {
			suri = string("file://") + string(cwdbuf) + suri.substr(1);
		}
	} else if (suri[0] == '/') {
		suri = std::string("file://") + suri;
	} else if (suri[0] == '~') {
#ifdef WIN32
		suri = string("file://") + string(std::getenv("HOMEDRIVE")) + string(std::getenv("HOMEPATH")) + suri.substr(1);
#else
		suri = string("file://") + string(std::getenv("HOME")) + suri.substr(1);
#endif
	}

#ifdef HAVE_URIPARSESINGLE
	const char *errpos;
	if (uriParseSingleUriA(&uri, puri, &errpos) != URI_SUCCESS) {
#else
	UriParserStateA uris;
	uris.uri = &uri;
	if (uriParseUriA(&uris, suri.c_str()) != URI_SUCCESS) {
#endif
		m_valid = false;
		m_host = "none";
		m_port = -1;
		m_proto = SCHEME_NONE;
		m_path = "";
		m_frag = "";
	} else {
		m_host = std::string(uri.hostText.first, uri.hostText.afterLast - uri.hostText.first);

		std::string prototext = std::string(uri.scheme.first, uri.scheme.afterLast - uri.scheme.first);
		if (prototext == "tcp") m_proto = SCHEME_TCP;
		else if (prototext == "udp") m_proto = SCHEME_UDP;
		else if (prototext == "ftl") m_proto = SCHEME_FTL;
		else if (prototext == "http") m_proto = SCHEME_HTTP;
		else if (prototext == "ws") m_proto = SCHEME_WS;
		else if (prototext == "ipc") m_proto = SCHEME_IPC;
		else if (prototext == "device") m_proto = SCHEME_DEVICE;
		else if (prototext == "file") m_proto = SCHEME_FILE;
		else if (prototext == "group") m_proto = SCHEME_GROUP;
		else m_proto = SCHEME_OTHER;
		m_protostr = prototext;

		std::string porttext = std::string(uri.portText.first, uri.portText.afterLast - uri.portText.first);
		m_port = atoi(porttext.c_str());
		m_userinfo = std::string(uri.userInfo.first, uri.userInfo.afterLast - uri.userInfo.first);

		for (auto h=uri.pathHead; h!=NULL; h=h->next) {
			auto pstr = std::string(
					h->text.first, h->text.afterLast - h->text.first);

			m_path += "/";
			m_path += pstr;
			m_pathseg.push_back(pstr);
		}

		//string query = std::string(uri.query.first, uri.query.afterLast - uri.query.first);
		if (uri.query.afterLast - uri.query.first > 0) {
			UriQueryListA *queryList;
			int itemCount;
			if (uriDissectQueryMallocA(&queryList, &itemCount, uri.query.first,
					uri.query.afterLast) != URI_SUCCESS) {
				// Failure
			}

			UriQueryListA *item = queryList;
			while (item) {
				m_qmap[item->key] = item->value;
				item = item->next;
			}
			uriFreeQueryListA(queryList);
		}

		uriFreeUriMembersA(&uri);

		auto fraglast = (uri.query.first != NULL) ? uri.query.first : uri.fragment.afterLast;
		if (uri.fragment.first != NULL && fraglast - uri.fragment.first > 0) {
			m_frag = std::string(uri.fragment.first, fraglast - uri.fragment.first);
		}

		m_valid = m_proto != SCHEME_NONE && (m_host.size() > 0 || m_path.size() > 0);

		if (m_valid) {
			// remove userinfo from base uri
			const char *start = uri.scheme.first;
			if (m_userinfo != "") {
				m_base = std::string(start, uri.userInfo.first - start);
				start = uri.userInfo.afterLast + 1;
			}
			else {
				m_base = std::string("");
			}
			if (m_qmap.size() > 0) {
				m_base += std::string(start, uri.query.first - start - 1);
			}
			else if (uri.fragment.first != NULL) {
				m_base += std::string(start, uri.fragment.first - start - 1);
			}
			else if (start) {
				m_base += std::string(start);
			}
			else {
				m_base += std::string("");
			}
		}
	}
}

string URI::to_string() const {
	return (m_qmap.size() > 0) ? m_base + "?" + getQuery() : m_base;
}

string URI::getPathSegment(int n) const {
	size_t N = (n < 0) ? m_pathseg.size()+n : n;
	if (N < 0 || N >= m_pathseg.size()) return "";
	else return m_pathseg[N];
}

string URI::getBaseURI(int n) const {
	if (n >= (int)m_pathseg.size()) return m_base;
	if (n >= 0) {
		string r = m_protostr + string("://") + m_host + ((m_port != 0) ? string(":") + std::to_string(m_port) : "");
		for (int i=0; i<n; i++) {
			r += "/";
			r += getPathSegment(i);
		}

		return r;
	} else if (m_pathseg.size()+n >= 0) {
		string r = m_protostr + string("://") + m_host + ((m_port != 0) ? string(":") + std::to_string(m_port) : "");
		size_t N = m_pathseg.size()+n;
		for (size_t i=0; i<N; i++) {
			r += "/";
			r += getPathSegment(static_cast<int>(i));
		}

		return r;
	} else return "";
}

std::string URI::getBaseURIWithUser() const {
	std::string result;

	result += m_protostr + "://";
	if (m_userinfo.size() > 0) {
		result += getUserInfo();
		result += "@";
	}
	result += m_host;
	if (m_port > 0) result += std::string(":") + std::to_string(m_port);
	result += m_path;
	return result;
}

string URI::getQuery() const {
	string q;
	for (auto x : m_qmap) {
		if (q.length() > 0) q += "&";
		q += x.first + "=" + x.second;
	}
	return q;
};

void URI::setAttribute(const string &key, const string &value) {
	m_qmap[key] = value;
}

void URI::setAttribute(const string &key, int value) {
	m_qmap[key] = std::to_string(value);
}

void URI::to_json(nlohmann::json &json) const {
	std::string uri = to_string();
	if (m_frag.size() > 0) uri += std::string("#") + getFragment();

	json["uri"] = uri;
	for (auto i : m_qmap) {
		auto *current = &json;

		size_t pos = 0;
		size_t lpos = 0;
		while ((pos = i.first.find('/', lpos)) != std::string::npos) {
			std::string subobj = i.first.substr(lpos, pos-lpos);
			current = &((*current)[subobj]);
			lpos = pos+1;
		}

		std::string obj = i.first.substr(lpos);

		auto p = nlohmann::json::parse(i.second, nullptr, false);
		if (!p.is_discarded()) {
			(*current)[obj] = p;
		} else {
			(*current)[obj] = i.second;
		}
	}
}

bool URI::hasUserInfo() const {
	return m_userinfo != "";
}

const std::string &URI::getUserInfo() const {
	return m_userinfo;
}
