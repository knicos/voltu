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
#pragma comment(lib, "User32.lib")
#else
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <iostream>

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

static bool is_file(const std::string &path) {
#ifdef WIN32
	WIN32_FIND_DATA ffd;
	HANDLE hFind = FindFirstFile(path.c_str(), &ffd);

	if (hFind == INVALID_HANDLE_VALUE) return false;
	FindClose(hFind);
	return true;
#else
	struct stat s;
	if (::stat(path.c_str(), &s) == 0) {
		return true;
	} else {
		return false;
	}
#endif
}

static std::string locateLibrary()
{
	// TODO: Use full paths and find correct versions
#if defined(WIN32)
	return "voltu.dll";
#else
	std::string name = "libvoltu.so";
	std::string vname = name + std::string(".") + std::to_string(VOLTU_VERSION_MAJOR) + std::string(".") + std::to_string(VOLTU_VERSION_MINOR);
	if (is_file(std::string("./") + vname)) return std::string("./") + vname;
	else if (is_file(std::string("./") + name)) return std::string("./") + name;
	else if (is_file(std::string("../") + vname)) return std::string("../") + vname;
	else if (is_file(std::string("../") + name)) return std::string("../") + name;
	else if (is_file(std::string("/usr/local/lib/") + vname)) return std::string("/usr/local/lib/") + vname;
	else if (is_file(std::string("/usr/local/lib/") + name)) return std::string("/usr/local/lib/") + name;
	return name;
#endif
}

std::shared_ptr<voltu::System> voltu::instance()
{
	if (g_init) return nullptr;
	
	std::string name = locateLibrary();
	std::cout << "Loading VolTu Runtime: " << name << std::endl;
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
