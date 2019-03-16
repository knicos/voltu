#include <ftl/disparity.hpp>

using ftl::Disparity;

std::map<std::string,std::function<Disparity*()>> Disparity::algorithms__;

Disparity::Disparity() : min_disp_(0), max_disp_(208) {}

Disparity *Disparity::create(const std::string &n) {
	if (algorithms__.count(n) != 1) return nullptr;
	return algorithms__[n]();
}

void Disparity::_register(const std::string &n, std::function<Disparity*()> f) {
	algorithms__[n] = f;
}

