/*
Copyright 2016 Fixstars Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef SGM_PATH_AGGREGATION_HPP
#define SGM_PATH_AGGREGATION_HPP

#include "device_buffer.hpp"
#include "types.hpp"

namespace sgm {

template <size_t MAX_DISPARITY>
class PathAggregation {

private:
	static const unsigned int NUM_PATHS = 8;

	DeviceBuffer<cost_type> m_cost_buffer;
	cudaStream_t m_streams[NUM_PATHS];
	cudaEvent_t m_events[NUM_PATHS];
	cudaEvent_t m_event;

public:
	PathAggregation();
	~PathAggregation();

	const cost_type *get_output() const {
		return m_cost_buffer.data();
	}

	void enqueue(
		const feature_type *left,
		const feature_type *right,
		int width,
		int height,
		unsigned int p1,
		const uint8_t *p2,
		int p2_pitch,
		const uint8_t* w,
		int w_pitch,
		int min_disp,
		cudaStream_t stream);

};

}

#endif
