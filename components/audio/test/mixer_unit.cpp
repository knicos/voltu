#include "catch.hpp"
#include <ftl/audio/mixer.hpp>

using ftl::audio::StereoMixerF;

TEST_CASE("Audio Mixer Stereo Float", "") {
	SECTION("Add two in sync tracks") {
		StereoMixerF<100> mixer(2);

		// Three 960 sample stereo frames
		std::vector<float> in1(960*2*3);
		std::vector<float> in2(960*2*3);

		for (int i=0; i<960*2*3; ++i) in1[i] = float(i)+1.0f;
		for (int i=0; i<960*2*3; ++i) in2[i] = float(i)+2.0f;

		mixer.write(0, in1);
		mixer.write(1, in2);
		mixer.mix();

		REQUIRE( mixer.writePosition() == 3 );
		REQUIRE( mixer.readPosition() == 0 );

		// Read one of the three valid frames
		std::vector<float> out;
		mixer.read(out, 1);
		bool correct = true;

		// Check all values are correct
		for (int i=0; i<960*2*1; ++i) {
			float e = float(i)+1.0f + float(i)+2.0f;
			correct &= int(e) == int(out[i]);
		}

		REQUIRE( correct );
	}

	SECTION("Add two out of sync tracks") {
		StereoMixerF<100> mixer(2);

		// Three 960 sample stereo frames
		std::vector<float> in1(960*2*3);
		std::vector<float> in2(960*2*2);

		for (int i=0; i<960*2*3; ++i) in1[i] = float(i)+1.0f;
		for (int i=0; i<960*2*2; ++i) in2[i] = float(i)+2.0f;

		mixer.write(0, in1);
		mixer.write(1, in2);
		mixer.mix();

		REQUIRE( mixer.writePosition() == 2 );
		REQUIRE( mixer.readPosition() == 0 );

		// Read one of the three valid frames
		std::vector<float> out;
		mixer.read(out, 2);
		bool correct = true;

		// Check all values are correct
		for (int i=0; i<960*2*2; ++i) {
			float e = float(i)+1.0f + float(i)+2.0f;
			correct &= int(e) == int(out[i]);
		}

		REQUIRE( correct );

		// Now add final frame
		std::vector<float> in3(960*2*1);
		for (int i=0; i<960*2*1; ++i) in3[i] = float(i)+1.0f;

		mixer.write(1, in3);
		mixer.mix();

		REQUIRE( mixer.writePosition() == 3 );
		REQUIRE( mixer.readPosition() == 2 );

		mixer.read(out, 1);

		// Check all values are correct
		for (int i=0; i<960*2*1; ++i) {
			float e = float(i)+1.0f + float(i+960*2*2)+1.0f;
			correct &= int(e) == int(out[i]);
		}

		REQUIRE( correct );
	}
}

TEST_CASE("Audio Mixer Stereo Float Dynamic Tracks", "") {
	SECTION("Add one track after write") {
		StereoMixerF<100> mixer(1);

		// Three 960 sample stereo frames
		std::vector<float> in1(960*2*3);
		for (int i=0; i<960*2*3; ++i) in1[i] = float(i)+1.0f;

		mixer.write(0, in1);
		mixer.mix();

		REQUIRE( mixer.writePosition() == 3 );
		REQUIRE( mixer.readPosition() == 0 );

		std::vector<float> in2(960*2*3);
		for (int i=0; i<960*2*3; ++i) in2[i] = float(i)+2.0f;

		mixer.resize(2);
		mixer.write(0, in1);
		mixer.write(1, in2);
		mixer.mix();

		REQUIRE( mixer.writePosition() == 6 );
		REQUIRE( mixer.readPosition() == 0 );
		REQUIRE( mixer.frames() == 6 );

		// Read one of the three valid frames
		std::vector<float> out;
		mixer.read(out, mixer.frames());
		bool correct = true;

		// Check all values are correct
		for (int i=0; i<960*2*3; ++i) {
			float e = float(i)+1.0f;
			correct &= int(e) == int(out[i]);
		}
		for (int i=960*2*3; i<960*2*6; ++i) {
			float e = float(i-960*2*3)+1.0f + float(i-960*2*3)+2.0f;
			correct &= int(e) == int(out[i]);
		}

		REQUIRE( correct );
	}
}
