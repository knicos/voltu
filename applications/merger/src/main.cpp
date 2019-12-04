#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/codecs/writer.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/hevc.hpp>

#include <fstream>

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "merger_default");

	std::string outputfile = root->value("out", std::string("output.ftl"));
	std::vector<std::string> paths = *root->get<std::vector<std::string>>("paths");
	int timeoff = int(root->value("offset", 0.0f) * 1000.0f);
	int stream_mask1 = root->value("mask1",0xFF);
	int stream_mask2 = root->value("mask2",0xFF);

	if (paths.size() == 0) {
		LOG(ERROR) << "Missing input ftl file(s).";
		return -1;
	}

	// Generate the output writer...
	std::ofstream of;
	of.open(outputfile);
	if (!of.is_open()) {
		LOG(ERROR) << "Could not open output file: " << outputfile;
		return -1;
	}

	ftl::codecs::Writer out(of);
	out.begin();

	std::vector<std::ifstream> fs;
	std::vector<ftl::codecs::Reader*> rs;
	fs.resize(paths.size());
	rs.resize(paths.size());

	for (size_t i=0; i<paths.size(); ++i) {
		fs[i].open(paths[i]);
		if (!fs[i].is_open()) {
			LOG(ERROR) << "Could not open file: " << paths[i];
			return -1;
		}

		LOG(INFO) << "Opening("<< i <<"): " << paths[i];

		rs[i] = new ftl::codecs::Reader(fs[i]);
		if (!rs[i]->begin()) {
			LOG(ERROR) << "Bad ftl file format";
			return -1;
		}
	}

	std::map<int,int> idmap;
	int lastid = 0;

	bool res = rs[0]->read(90000000000000, [&rs,&out,&idmap,&lastid,stream_mask1,stream_mask2,timeoff](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		if (((0x1 << spkt.streamID) & stream_mask1) == 0) return;

		ftl::codecs::StreamPacket spkt2 = spkt;
		if (idmap.find(spkt.streamID) == idmap.end()) {
			idmap[spkt.streamID] = lastid++;
		}
		spkt2.streamID = idmap[spkt.streamID];

		// Now read all other sources up to the same packet timestamp.
		out.write(spkt2, pkt);

		for (size_t j=1; j<rs.size(); ++j) {
			ftl::codecs::Reader *r = rs[j];

			// FIXME: Need to truncate other stream if the following returns
			// no frames, meaning the timeshift causes this stream to run out
			// before the main stream.
			rs[j]->read(spkt.timestamp+timeoff+1, [&out,&idmap,&lastid,j,r,stream_mask2,timeoff](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
				if (((0x1 << spkt.streamID) & stream_mask2) == 0) return;
				if (int(spkt.channel) < 32 && spkt.timestamp < r->getStartTime()+timeoff) return;

				ftl::codecs::StreamPacket spkt2 = spkt;
				if (idmap.find(spkt.streamID + (j << 16)) == idmap.end()) {
					idmap[spkt.streamID+(j << 16)] = lastid++;
				}
				spkt2.streamID = idmap[spkt.streamID + (j << 16)];
				spkt2.timestamp -= timeoff;

				out.write(spkt2, pkt);
			});
		}
	});

	out.end();
	of.close();

	for (size_t i=0; i<rs.size(); ++i) {
		rs[i]->end();
		delete rs[i];
		fs[i].close();
	}

	return 0;
}