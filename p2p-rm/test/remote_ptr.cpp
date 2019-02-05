#include "catch.hpp"
#include <ftl/p2p-rm/remote_ptr.hpp>
#include <memory.h>

// Mock the BLOB
static bool is_finished = false;
void ftl::rm::Blob::finished() {
	is_finished = true;
}

static bool blob_size;
static const int *blob_data;
void ftl::rm::Blob::write(size_t offset, const char *data, size_t size) {
	blob_size = size;
	blob_data = (const int*)data;
	memcpy(data_+offset,data,size);
}

void ftl::rm::Blob::read(size_t offset, char *data, size_t size) {
	memcpy(data,data_+offset,size);
}

SCENARIO( "Reading from a remote pointer", "[remote_ptr]" ) {
	// Make a dummy blob
	auto blob = new ftl::rm::Blob();
	blob->data_ = (char*)(new int[5]);
	((int*)(blob->data_))[0] = 55;
	((int*)(blob->data_))[1] = 66;
	
	GIVEN( "a valid POD const remote pointer" ) {
		const ftl::remote_ptr<int> pa{blob,0};
		REQUIRE( *pa == 55 );
		REQUIRE( pa[0] == 55 );
		REQUIRE( pa[1] == 66 );
	}
}

SCENARIO( "Writing to a remote pointer", "[remote_ptr]" ) {
	// Make a dummy blob
	auto blob = new ftl::rm::Blob();
	blob->data_ = (char*)(new int[5]);
	((int*)(blob->data_))[0] = 55;
	((int*)(blob->data_))[1] = 66;
	
	GIVEN( "a valid POD remote pointer" ) {
		ftl::remote_ptr<int> pa{blob,0};
		is_finished = false;
		*pa = 23;
		REQUIRE( *pa == 23 );
		REQUIRE( is_finished );
		REQUIRE( pa[0] == 23 );
		pa[1] = 25;
		REQUIRE( pa[1] == 25 );
	}
	
	GIVEN( "a persistent write_ref" ) {
		ftl::remote_ptr<int> pa{blob,0};
		is_finished = false;
		auto ra = *pa;
		
		ra = 23;
		REQUIRE( ra == 23 );
		REQUIRE( !is_finished );
		ra.reset();
		REQUIRE( is_finished );
	}
}

SCENARIO( "Writing to readonly pointer fails", "[remote_ptr]" ) {
	// Make a dummy blob
	auto blob = new ftl::rm::Blob();
	blob->data_ = (char*)(new int[5]);
	((int*)(blob->data_))[0] = 55;
	((int*)(blob->data_))[1] = 66;
	
	GIVEN( "a valid POD const remote pointer" ) {
		const ftl::remote_ptr<int> pa{blob,0};
		*pa = 23;
		REQUIRE( *pa == 55 );
	}
}

