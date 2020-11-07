/**
 * @file voltu.cpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#include <voltu/initialise.hpp>
#include <voltu/types/errors.hpp>
#include <voltu/voltu.hpp>

#if defined(WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

static bool g_init = false;

typedef void* Library;

static Library loadLibrary(const char *file)
{
#if defined(WIN32)
	return (Library)LoadLibraryEx(file, NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
#else
	return (Library)dlopen(file, RTLD_LOCAL | RTLD_NOW);
#endif
}

static void *getFunction(Library lib, const char *fname)
{
#if defined(WIN32)
	return (void*)GetProcAddress((HMODULE)lib, fname);
#else
	return dlsym(lib, fname);
#endif
}


static void unloadLibrary(Library lib)
{
	if (!lib) return;

#if defined(WIN32)
	FreeLibrary((HMODULE)lib);
#else
	dlclose(lib);
#endif
}

static std::string locateLibrary()
{
	// TODO: Use full paths and find correct versions
#if defined(WIN32)
	return "voltu.dll";
#else
	return "libvoltu.so";
#endif
}

std::shared_ptr<voltu::System> voltu::instance()
{
	if (g_init) return nullptr;
	
	std::string name = locateLibrary();
	Library handle = loadLibrary(name.c_str());

	if (handle)
	{
		voltu::System* (*init)();
		init = (voltu::System* (*)())getFunction(handle, "voltu_initialise");

		if (init)
		{
			g_init = true;
			std::shared_ptr<voltu::System> instance(init());

			if (!instance)
			{
				throw voltu::exceptions::RuntimeAlreadyInUse();
			}

			// FIXME: Perhaps use a C method instead for safety?
			auto ver = instance->getVersion();
			if (ver.major != VOLTU_VERSION_MAJOR || ver.minor != VOLTU_VERSION_MINOR)
			{
				throw voltu::exceptions::RuntimeVersionMismatch();
			}

			return instance;
		}
		else
		{
			throw voltu::exceptions::LibraryLoadFailed();
		}
	}
	else
	{
		throw voltu::exceptions::LibraryLoadFailed();
	}

	return nullptr;
}
