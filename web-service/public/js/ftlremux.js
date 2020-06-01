var ee = require('event-emitter');
const MUXJS = require('mux.js');
const MP4 = require('./lib/mp4-generator');
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

function concatAudioSamples(samples) {
	let totallen = 0;
	for (let i=0; i<samples.length; ++i) {
		totallen += samples[i].size;
	}

	let result = new Uint8Array(totallen);
	let offset = 0;
	for (let i=0; i<samples.length; ++i) {
		result.set(samples[i].data, offset);
		offset += samples[i].size;
	}
	return MP4.mdat(result);
}

function reformAudio(data) {
	let offset = 0;
	let results = [];

	while (offset < data.length) {
		let l = data[offset] + (data[offset+1] << 8); //view.getInt16(offset);
		offset += 2;
		//console.log("Opus frame code = ", data[offset] & 0x03, l);
		//let p;
		let p = data.subarray(offset, offset+l);
		/*let ll = l-1;  // Remove config byte
		if (ll <= 251) {
			p = new Uint8Array(l+1);
			p[0] = data[offset];
			p[1] = ll & 0xff; 
			p.set(data.subarray(offset+1, offset+l), 2);
		} else {
			//let p = data.subarray(offset, offset+l);
			p = new Uint8Array(l+2);
			p[0] = data[offset];
			let l2 = (ll-252) >> 2;
			let l1 = 252 + ((ll-252) - (l2 << 2));
			p[1] = l1; 
			p[3] = l2;
			console.log("Opus size", l1 + 4*l2, ll, l1, l2);
			p.set(data.subarray(offset+1, offset+l), 3);
		}*/
		//let mdat = MP4.mdat(p);
		results.push({size: p.byteLength, duration: 1800, data: p});
		offset += l;
	}

	return results;
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

	this.audiotrack = {
		timelineStartInfo: {
			baseMediaDecodeTime: 0
		},
		baseMediaDecodeTime: 1800,
		id: 1,
		codec: 'opus',
		type: 'audio',
		samples: [{
			size: 0,
			duration: 1800 //960
		}],
		duration: 0,
		insamplerate: 48000,
		channelcount: 2,
		width: 0,
		height: 0
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
			console.log("Init", this.track);
			if (this.has_audio) {
				this.emit('data', MP4.initSegment([this.track, this.audiotrack]));
			} else {
				this.emit('data', MP4.initSegment([this.track]));
			}
			this.init_seg = true;
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

	this.sequenceNo = 0;
	this.audioSequenceNo = 0;
	this.seen_keyframe = false;
	this.ts = 0;
	this.dts = 0;
	this.init_seg = false;
	this.init_audio = false;
	this.has_audio = false;
};

ee(FTLRemux.prototype);

FTLRemux.prototype.push = function(spkt, pkt) {
	if (this.paused || !this.active) {
		return;
	}

	if (pkt[0] === 33) {  // Opus audio
		if (this.has_audio && this.init_seg) {
			// Split into individual packets and create moof+mdat
			let samples = reformAudio(pkt[5]);
			this.audiotrack.samples = samples;

			// TODO: Can this audio track be combined into same fragment as video frame?
			let moof = MP4.moof(this.audioSequenceNo++, [this.audiotrack]);
			let mdat = concatAudioSamples(samples);
			let result = new Uint8Array(moof.byteLength + mdat.byteLength);
			result.set(moof);
			result.set(mdat, moof.byteLength);
			this.emit('data', result);
			this.audiotrack.baseMediaDecodeTime += 1800*samples.length; // 1800 = 20ms*90 or frame size 960@48000hz in 90000 ticks/s
		}
	} else if(pkt[0] === 2){  // H264 packet.
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
