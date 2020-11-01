#ifndef _CUDATL_HOST_UTILITY_HPP_
#define _CUDATL_HOST_UTILITY_HPP_

#include <cuda_runtime.hpp>
#include <string>

namespace cudatl {

inline safeCall(cudaError_t e) {
	if (e != cudaSuccess) throw new std::exception(std::string("Cuda Error "+std::to_string(int(e))));
}

}

#endif