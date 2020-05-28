var ee = require('event-emitter');
const MUXJS = require('mux.js');
const MP4 = MUXJS.mp4.generator;
const H264Stream = MUXJS.codecs.h264.H264Stream;

const VIDEO_PROPERTIES = [
	'width',
	'height',
	'profileIdc',
	'levelIdc',
	'profileCompatibility',
	'sarRatio'
  ];

function getNALType(data) {
	return (data.length > 4) ? data.readUInt8(4) & 0x1F : 0;
}

function isKeyFrame(data) {
	return getNALType(data) == 7;  // SPS
}

function concatNals(sample) {
	let length = sample.size;
	let data = new Uint8Array(length);
	let view = new DataView(data.buffer);
	let dataOffset = 0;

	for (var i=0; i<sample.units.length; ++i) {
		view.setUint32(dataOffset, sample.units[i].data.byteLength);
        dataOffset += 4;
        data.set(sample.units[i].data, dataOffset);
        dataOffset += sample.units[i].data.byteLength;
	}

	sample.data = data;
}

var createDefaultSample = function() {
	return {
	  units: [],
	  data: null,
	  size: 0,
	  compositionTimeOffset: 1,
	  duration: 0,
	  dataOffset: 0,
	  flags: {
		isLeading: 0,
		dependsOn: 1,
		isDependedOn: 0,
		hasRedundancy: 0,
		degradationPriority: 0,
		isNonSyncSample: 1
	  },
	  keyFrame: true
	};
  };

/**
 * Convert FTL stream packets into MP4 fragments for use with MSE. It emits
 * 'data' events with a single argument containing the MP4 fragment.
 */
function FTLRemux() {
	this.frameset = 0;
	this.source = 0;
	this.channel = 0;
	this.paused = false;
	this.active = false;

	this.track = {
		timelineStartInfo: {
			baseMediaDecodeTime: 0
		},
		baseMediaDecodeTime: 0,
		id: 0,
		codec: 'avc',
		type: 'video',
		samples: [],
		duration: 0
	};

	this.h264 = new H264Stream();

	this.h264.on('data', (nalUnit) => {
		// record the track config
		if (nalUnit.nalUnitType === 'seq_parameter_set_rbsp') {
			this.track.config = nalUnit.config;
			this.track.sps = [nalUnit.data];

			VIDEO_PROPERTIES.forEach(function(prop) {
				this.track[prop] = nalUnit.config[prop];
			}, this);
		}

		if (nalUnit.nalUnitType === 'pic_parameter_set_rbsp') {
			//pps = nalUnit.data;
			this.track.pps = [nalUnit.data];
		}

		if (!this.init_seg && this.track.sps && this.track.pps) {
			this.init_seg = true;
			console.log("Init", this.track);
			this.emit('data', MP4.initSegment([this.track]));
		}

		let keyFrame = nalUnit.nalUnitType == 'slice_layer_without_partitioning_rbsp_idr';
		let sample = this.track.samples[0];
		sample.units.push(nalUnit);
		sample.size += nalUnit.data.byteLength + 4;

		sample.keyFrame &= keyFrame;
		
		if (keyFrame) {
			sample.flags.isNonSyncSample = 0;
			sample.flags.dependsOn = 2;
		}
	});

	this.mime = 'video/mp4; codecs="avc1.640028"';
	this.sequenceNo = 0;
	this.seen_keyframe = false;
	this.ts = 0;
	this.dts = 0;
	this.init_seg = false;
};

ee(FTLRemux.prototype);

FTLRemux.prototype.push = function(spkt, pkt) {
	if (this.paused || !this.active) {
		return;
	}

	if(pkt[0] === 2){  // H264 packet.
		if (spkt[1] == this.frameset && spkt[2] == this.source && spkt[3] == this.channel) {

			if (!this.seen_keyframe) {
				if (isKeyFrame(pkt[5])) {
					console.log("Key frame ", spkt[0]);
					this.seen_keyframe = true;
				}
			}
		
			if (this.seen_keyframe) {
				if (this.ts == 0) this.ts = spkt[0];
				//if (this.track.samples.length > 0) console.error("Unfinished sample");
				this.dts += spkt[0]-this.ts;

				this.track.samples.push(createDefaultSample());

				this.h264.push({
					type: 'video',
					dts: this.dts,
					pts: spkt[0],
					data: pkt[5],
					trackId: 0
				});
				this.h264.flush();

				let sample = this.track.samples[0];
				concatNals(sample);
				let delta = (spkt[0]-this.ts)*90;
				sample.duration = (delta > 0) ? delta : 1000;

				let moof = MP4.moof(this.sequenceNo++, [this.track]);
				let mdat = MP4.mdat(sample.data);
				let result = new Uint8Array(moof.byteLength + mdat.byteLength);
				//result.set(MP4.STYP);
				result.set(moof);
				result.set(mdat, moof.byteLength);
				this.emit('data', result);

				this.track.samples = [];
				this.track.baseMediaDecodeTime += delta;

				this.ts = spkt[0];
			}
		}
	}
}

FTLRemux.prototype.select = function(frameset, source, channel) {
	this.frameset = frameset;
	this.source = source;
	this.channel = channel;

	this.reset();
}

FTLRemux.prototype.reset = function() {
	this.init_seg = false;
	this.seen_keyframe = false;
	this.ts = 0;
	this.track.baseMediaDecodeTime = 0;
	this.sequenceNo = 0;
	this.active = true;
}

module.exports = FTLRemux;
