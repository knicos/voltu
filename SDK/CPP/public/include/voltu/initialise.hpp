/**
 * @file initialise.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once
#include <memory>
#include <voltu/system.hpp>
#include "defines.hpp"

namespace voltu
{
	/**
	 * @brief Get core VolTu instance.
	 * 
	 * This method returns a smart pointer to a singleton VolTu runtime
	 * instance and must be the first VolTu call. On any given machine it is
	 * only sensible and possible to have one runtime instance of VolTu due to
	 * its use of hardware devices. Multiple real instances are not possible.
	 * 
	 * @code
	 * int main(int argc, char** argv) {
	 *     auto vtu = voltu::instance();
	 * 
	 *     vtu->open("device:camera");
	 *     ...
	 * }
	 * @endcode
	 * 
	 * @note
	 * This method must only be called once.
	 * 
	 * @throw voltu::exceptions::LibraryLoadFailed
	 * If runtime not found or is invalid.
	 *
	 * @throw voltu::exceptions::RuntimeVersionMismatch
	 * If major or minor version does not match the SDK headers.
	 * 
	 * @throw voltu::exceptions::RuntimeAlreadyInUse
	 * If a runtime instance is in use by another application.
	 * 
	 * @return Singleton VolTu runtime instance.
	 */
	PY_API std::shared_ptr<voltu::System> instance();
}
