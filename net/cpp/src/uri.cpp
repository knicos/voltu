#include <ftl/uri.hpp>

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
}

void URI::_parse(uri_t puri) {
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

        m_valid = m_proto != SCHEME_NONE && m_host.size() > 0;

        if (m_valid) {
            if (m_qmap.size() > 0) m_base = std::string(uri.scheme.first, uri.query.first - uri.scheme.first - 1);
            else m_base = std::string(uri.scheme.first);
        }
    }
}

string URI::to_string() const {
    return (m_qmap.size() > 0) ? m_base + "?" + getQuery() : m_base;
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
