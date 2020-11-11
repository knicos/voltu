#pragma once

#include <voltu/types/image.hpp>
#include 
#include <memory>

namespace voltu
{

/**
 * @brief CUDA Processing Stream.
 * 
 * An instance of this class is mapped to a single CUDA stream, so any of the
 * available operations will occur within that stream. It is therefore
 * necessary to call `waitCompletion` after all steps have been finished.
 */
class CUDAProc
{
public:
	virtual bool waitCompletion(int timeout, bool except=false) = 0;

	virtual void* getInternalStream() = 0;

	virtual void visualiseDepthEnhancement(const voltu::ImagePtr &gt, const voltu::ImagePtr &depth_old, const voltu::ImagePtr &depth_new, const voltu::ImagePtr &colour) = 0;
};

}
