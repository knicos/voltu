#pragma once

#include "../defines.hpp"
#include <memory>
#include <string>

namespace voltu
{

class Property
{
public:
	virtual ~Property() = default;

	PY_API virtual void setInt(int) = 0;

	PY_API virtual void setFloat(float) = 0;

	PY_API virtual void setString(const std::string &) = 0;

	PY_API virtual void setBool(bool) = 0;

	template <typename T>
	inline void setEnum(T e) { setInt(static_cast<int>(e)); }

	PY_API virtual int getInt() = 0;

	PY_API virtual float getFloat() = 0;

	PY_API virtual std::string getString() = 0;

	PY_API virtual bool getBool() = 0;

	template <typename T>
	inline T getEnum() { return static_cast<int>(getInt()); }
};

typedef std::shared_ptr<Property> PropertyPtr;

}
