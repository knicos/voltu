#include <ftl/net/p2p.hpp>

using ftl::net::P2P;
using std::optional;
using std::tuple;
using ftl::UUID;
using std::get;
using namespace std::chrono;
using std::vector;
using std::string;

P2P::P2P(const char *uri) : Protocol(uri) {
	_registerRPC();
}

P2P::P2P(const string &uri) : Protocol(uri) {
	_registerRPC();
}

void P2P::_registerRPC() {
	bind_find_one("ping", &P2P::_ping);
}

vector<string> P2P::getAddresses(const UUID &peer) {
	vector<string> results;
	return results;
}

optional<long int> P2P::ping(const UUID &peer) {
	long int time = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
	auto p = find_one<long int>("ping", peer, time);
	
	if (!p) return {};
	return *p - time;
}

optional<long int> P2P::_ping(const UUID &peer, long int time) {
	if (id() == peer) {
		return time;
	} else {
		return {};
	}
}

