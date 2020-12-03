/**
 * @file string.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_UTILITY_STRING_HPP_
#define _FTL_UTILITY_STRING_HPP_

/**
 * Numeric value to string fix specified precision.
 */
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

#endif