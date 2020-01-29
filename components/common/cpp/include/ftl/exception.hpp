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

class exception : public std::exception
{
	public:
	explicit exception(const char *msg) : msg_(msg) {}
	explicit exception(const Formatter &msg) : msg_(msg.str()) {}

	const char * what () const throw () {
    	return msg_.c_str();
    }

	private:
	std::string msg_;
};
}

#define FTL_Error(A) (ftl::exception(ftl::Formatter() << __FILE__ << ":" << __LINE__ << ": " << A))

#endif  // _FTL_EXCEPTION_HPP_
