#pragma once

#include <exception>

namespace voltu
{
namespace exceptions
{

struct Exception : public std::exception
{
	virtual const char* what() const noexcept { return "VolTu General Error"; }
};

}
}

#define VOLTU_EXCEPTION(NAME,BASE,MSG) struct NAME : public voltu::exceptions::BASE { virtual const char* what() const noexcept { return MSG; } };

namespace voltu
{
namespace exceptions
{

VOLTU_EXCEPTION(BadImageChannel, Exception, "Invalid image channel");
VOLTU_EXCEPTION(NoFrame, Exception, "No frame available");
VOLTU_EXCEPTION(AlreadyInit, Exception, "VolTu already initialised");
VOLTU_EXCEPTION(LibraryLoadFailed, Exception, "Could not load VolTu library");
VOLTU_EXCEPTION(LibraryVersionMismatch, Exception, "Wrong version of library found");
VOLTU_EXCEPTION(BadSourceURI, Exception, "Bad source URI");
VOLTU_EXCEPTION(InvalidFrameObject, Exception, "Invalid Frame object");
VOLTU_EXCEPTION(InternalRenderError, Exception, "Internal renderer error");
VOLTU_EXCEPTION(InvalidProperty, Exception, "Unknown property enum");
VOLTU_EXCEPTION(BadParameterValue, Exception, "Method parameter is not valid");

}
}
