/**
 * @file exception.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_EXCEPTION_HPP_
#define _FTL_EXCEPTION_HPP_

#include <sstream>

namespace ftl {
class Formatter {
	public:
	Formatter() {}
	~Formatter() {}

	template <typename Type>
	inline Formatter & operator << (const Type & value)
	{
		stream_ << value;
		return *this;
	}

	inline std::string str() const         { return stream_.str(); }
	inline operator std::string () const   { return stream_.str(); }

	enum ConvertToString
	{
		to_str
	};
	inline std::string operator >> (ConvertToString) { return stream_.str(); }

private:
	std::stringstream stream_;

	Formatter(const Formatter &);
	Formatter & operator = (Formatter &);
};

/**
 * Main FTL internal exception class. Use via Macro below.
 */
class exception : public std::exception
{
	public:
	explicit exception(const char *msg);
	explicit exception(const Formatter &msg);
	~exception();

	const char* what() const throw () {
		processed_ = true;
		return msg_.c_str();
	}

	std::string trace() const throw () {
		return decode_backtrace();
	}

	void ignore() const { processed_ = true; }

	private:
	std::string decode_backtrace() const;

	std::string msg_;
	mutable bool processed_;

#ifdef __GNUC__
	static const int TRACE_SIZE_MAX_ = 16;
	void* trace_[TRACE_SIZE_MAX_];
	int trace_size_;
#endif
};

}

#define FTL_Error(A) (ftl::exception(ftl::Formatter() << A << " [" << __FILE__ << ":" << __LINE__ << "]"))

#endif  // _FTL_EXCEPTION_HPP_
