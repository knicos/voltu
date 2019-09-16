#ifndef _FTL_EXCEPTION_HPP_
#define _FTL_EXCEPTION_HPP_

namespace ftl {
class exception : public std::exception
{
	public:
	explicit exception(const char *msg) : msg_(msg) {};

	const char * what () const throw () {
    	return msg_;
    }

	private:
	const char *msg_;
};
}

#endif  // _FTL_EXCEPTION_HPP_
