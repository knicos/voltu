"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var MP4 = (function () {
    function MP4() {
    }
    MP4.init = function () {
        MP4.initalized = true;
        MP4.types = {
            avc1: [],
            avcC: [],
            btrt: [],
            dinf: [],
            dref: [],
            esds: [],
            ftyp: [],
            hdlr: [],
            mdat: [],
            mdhd: [],
            mdia: [],
            mfhd: [],
            minf: [],
            moof: [],
            moov: [],
            mp4a: [],
            mvex: [],
            mvhd: [],
            sdtp: [],
            stbl: [],
            stco: [],
            stsc: [],
            stsd: [],
            stsz: [],
            stts: [],
            styp: [],
            tfdt: [],
            tfhd: [],
            traf: [],
            trak: [],
            trun: [],
            trep: [],
            trex: [],
            tkhd: [],
            vmhd: [],
            smhd: [],
        };
        for (var type in MP4.types) {
            if (MP4.types.hasOwnProperty(type)) {
                MP4.types[type] = [
                    type.charCodeAt(0),
                    type.charCodeAt(1),
                    type.charCodeAt(2),
                    type.charCodeAt(3),
                ];
            }
        }
        var hdlr = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x76, 0x69, 0x64, 0x65,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x56, 0x69, 0x64, 0x65,
            0x6f, 0x48, 0x61, 0x6e,
            0x64, 0x6c, 0x65, 0x72, 0x00,
        ]);
        var dref = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x0c,
            0x75, 0x72, 0x6c, 0x20,
            0x00,
            0x00, 0x00, 0x01,
        ]);
        var stco = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]);
        MP4.STTS = MP4.STSC = MP4.STCO = stco;
        MP4.STSZ = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]);
        MP4.VMHD = new Uint8Array([
            0x00,
            0x00, 0x00, 0x01,
            0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        ]);
        MP4.SMHD = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        ]);
        MP4.STSD = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01
        ]);
        MP4.FTYP = MP4.box(MP4.types.ftyp, new Uint8Array([
            0x69, 0x73, 0x6f, 0x35,
            0x00, 0x00, 0x00, 0x01,
            0x61, 0x76, 0x63, 0x31,
            0x69, 0x73, 0x6f, 0x35,
            0x64, 0x61, 0x73, 0x68,
        ]));
        MP4.STYP = MP4.box(MP4.types.styp, new Uint8Array([
            0x6d, 0x73, 0x64, 0x68,
            0x00, 0x00, 0x00, 0x00,
            0x6d, 0x73, 0x64, 0x68,
            0x6d, 0x73, 0x69, 0x78,
        ]));
        MP4.DINF = MP4.box(MP4.types.dinf, MP4.box(MP4.types.dref, dref));
        MP4.HDLR = MP4.box(MP4.types.hdlr, hdlr);
    };
    MP4.box = function (type) {
        var payload = [];
        for (var _i = 1; _i < arguments.length; _i++) {
            payload[_i - 1] = arguments[_i];
        }
        var size = 8;
        for (var _a = 0, payload_1 = payload; _a < payload_1.length; _a++) {
            var p = payload_1[_a];
            size += p.byteLength;
        }
        var result = new Uint8Array(size);
        result[0] = (size >> 24) & 0xff;
        result[1] = (size >> 16) & 0xff;
        result[2] = (size >> 8) & 0xff;
        result[3] = size & 0xff;
        result.set(type, 4);
        size = 8;
        for (var _b = 0, payload_2 = payload; _b < payload_2.length; _b++) {
            var box = payload_2[_b];
            result.set(box, size);
            size += box.byteLength;
        }
        return result;
    };
    MP4.mdat = function (data) {
        return MP4.box(MP4.types.mdat, data);
    };
    MP4.mdhd = function (timescale) {
        return MP4.box(MP4.types.mdhd, new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x02,
            (timescale >> 24) & 0xFF,
            (timescale >> 16) & 0xFF,
            (timescale >> 8) & 0xFF,
            timescale & 0xFF,
            0x00, 0x00, 0x00, 0x00,
            0x55, 0xc4,
            0x00, 0x00,
        ]));
    };
    MP4.mdia = function (track) {
        return MP4.box(MP4.types.mdia, MP4.mdhd(track.timescale), MP4.HDLR, MP4.minf(track));
    };
    MP4.mfhd = function (sequenceNumber) {
        return MP4.box(MP4.types.mfhd, new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            (sequenceNumber >> 24),
            (sequenceNumber >> 16) & 0xFF,
            (sequenceNumber >> 8) & 0xFF,
            sequenceNumber & 0xFF,
        ]));
    };
    MP4.minf = function (track) {
        return MP4.box(MP4.types.minf, MP4.box(MP4.types.vmhd, MP4.VMHD), MP4.DINF, MP4.stbl(track));
    };
    MP4.moof = function (sn, baseMediaDecodeTime, track) {
        return MP4.box(MP4.types.moof, MP4.mfhd(sn), MP4.traf(track, baseMediaDecodeTime));
    };
    MP4.moov = function (tracks, duration, timescale) {
        var boxes = [];
        for (var _i = 0, tracks_1 = tracks; _i < tracks_1.length; _i++) {
            var track = tracks_1[_i];
            boxes.push(MP4.trak(track));
        }
        return MP4.box.apply(MP4, [MP4.types.moov, MP4.mvhd(timescale, duration), MP4.mvex(tracks)].concat(boxes));
    };
    MP4.mvhd = function (timescale, duration) {
        var bytes = new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x02,
            (timescale >> 24) & 0xFF,
            (timescale >> 16) & 0xFF,
            (timescale >> 8) & 0xFF,
            timescale & 0xFF,
            (duration >> 24) & 0xFF,
            (duration >> 16) & 0xFF,
            (duration >> 8) & 0xFF,
            duration & 0xFF,
            0x00, 0x01, 0x00, 0x00,
            0x01, 0x00,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x40, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x02,
        ]);
        return MP4.box(MP4.types.mvhd, bytes);
    };
    MP4.mvex = function (tracks) {
        var boxes = [];
        for (var _i = 0, tracks_2 = tracks; _i < tracks_2.length; _i++) {
            var track = tracks_2[_i];
            boxes.push(MP4.trex(track));
        }
        return MP4.box.apply(MP4, [MP4.types.mvex].concat(boxes, [MP4.trep()]));
    };
    MP4.trep = function () {
        return MP4.box(MP4.types.trep, new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
        ]));
    };
    MP4.stbl = function (track) {
        return MP4.box(MP4.types.stbl, MP4.stsd(track), MP4.box(MP4.types.stts, MP4.STTS), MP4.box(MP4.types.stsc, MP4.STSC), MP4.box(MP4.types.stsz, MP4.STSZ), MP4.box(MP4.types.stco, MP4.STCO));
    };
    MP4.avc1 = function (track) {
        var sps = [];
        var pps = [];
        for (var _i = 0, _a = track.sps; _i < _a.length; _i++) {
            var data = _a[_i];
            var len = data.byteLength;
            sps.push((len >>> 8) & 0xFF);
            sps.push((len & 0xFF));
            sps = sps.concat(Array.prototype.slice.call(data));
        }
        for (var _b = 0, _c = track.pps; _b < _c.length; _b++) {
            var data = _c[_b];
            var len = data.byteLength;
            pps.push((len >>> 8) & 0xFF);
            pps.push((len & 0xFF));
            pps = pps.concat(Array.prototype.slice.call(data));
        }
        var avcc = MP4.box(MP4.types.avcC, new Uint8Array([
            0x01,
            sps[3],
            sps[4],
            sps[5],
            0xfc | 3,
            0xE0 | track.sps.length,
        ].concat(sps).concat([
            track.pps.length,
        ]).concat(pps)));
        var width = track.width;
        var height = track.height;
        return MP4.box(MP4.types.avc1, new Uint8Array([
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00,
            0x00, 0x01,
            0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            (width >> 8) & 0xFF,
            width & 0xff,
            (height >> 8) & 0xFF,
            height & 0xff,
            0x00, 0x48, 0x00, 0x00,
            0x00, 0x48, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x01,
            0x12,
            0x62, 0x69, 0x6E, 0x65,
            0x6C, 0x70, 0x72, 0x6F,
            0x2E, 0x72, 0x75, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00,
            0x00, 0x18,
            0x11, 0x11
        ]), avcc, MP4.box(MP4.types.btrt, new Uint8Array([
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x2d, 0xc6, 0xc0,
            0x00, 0x2d, 0xc6, 0xc0,
        ])));
    };
    MP4.stsd = function (track) {
        return MP4.box(MP4.types.stsd, MP4.STSD, MP4.avc1(track));
    };
    MP4.tkhd = function (track) {
        var id = track.id;
        var width = track.width;
        var height = track.height;
        return MP4.box(MP4.types.tkhd, new Uint8Array([
            0x00,
            0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x02,
            (id >> 24) & 0xFF,
            (id >> 16) & 0xFF,
            (id >> 8) & 0xFF,
            id & 0xFF,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00,
            (track.type === 'audio' ? 0x01 : 0x00), 0x00,
            0x00, 0x00,
            0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x40, 0x00, 0x00, 0x00,
            (width >> 8) & 0xFF,
            width & 0xFF,
            0x00, 0x00,
            (height >> 8) & 0xFF,
            height & 0xFF,
            0x00, 0x00,
        ]));
    };
    MP4.traf = function (track, baseMediaDecodeTime) {
        var id = track.id;
        return MP4.box(MP4.types.traf, MP4.box(MP4.types.tfhd, new Uint8Array([
            0x00,
            0x02, 0x00, 0x00,
            (id >> 24),
            (id >> 16) & 0XFF,
            (id >> 8) & 0XFF,
            (id & 0xFF),
        ])), MP4.box(MP4.types.tfdt, new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            (baseMediaDecodeTime >> 24),
            (baseMediaDecodeTime >> 16) & 0XFF,
            (baseMediaDecodeTime >> 8) & 0XFF,
            (baseMediaDecodeTime & 0xFF),
        ])), MP4.trun(track, 16 +
            16 +
            8 +
            16 +
            8 +
            8));
    };
    MP4.trak = function (track) {
        track.duration = track.duration || 0xffffffff;
        return MP4.box(MP4.types.trak, MP4.tkhd(track), MP4.mdia(track));
    };
    MP4.trex = function (track) {
        var id = track.id;
        return MP4.box(MP4.types.trex, new Uint8Array([
            0x00,
            0x00, 0x00, 0x00,
            (id >> 24),
            (id >> 16) & 0XFF,
            (id >> 8) & 0XFF,
            (id & 0xFF),
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x3c,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x00, 0x00,
        ]));
    };
    MP4.trun = function (track, offset) {
        var samples = track.samples || [];
        var len = samples.length;
        var additionalLen = track.isKeyFrame ? 4 : 0;
        var arraylen = 12 + additionalLen + (4 * len);
        var array = new Uint8Array(arraylen);
        offset += 8 + arraylen;
        array.set([
            0x00,
            0x00, 0x02, (track.isKeyFrame ? 0x05 : 0x01),
            (len >>> 24) & 0xFF,
            (len >>> 16) & 0xFF,
            (len >>> 8) & 0xFF,
            len & 0xFF,
            (offset >>> 24) & 0xFF,
            (offset >>> 16) & 0xFF,
            (offset >>> 8) & 0xFF,
            offset & 0xFF,
        ], 0);
        if (track.isKeyFrame) {
            array.set([
                0x00, 0x00, 0x00, 0x00,
            ], 12);
        }
        for (var i = 0; i < len; i++) {
            var sample = samples[i];
            var size = sample.size;
            array.set([
                (size >>> 24) & 0xFF,
                (size >>> 16) & 0xFF,
                (size >>> 8) & 0xFF,
                size & 0xFF,
            ], 12 + additionalLen + 4 * i);
        }
        return MP4.box(MP4.types.trun, array);
    };
    MP4.initSegment = function (tracks, duration, timescale) {
        if (!MP4.initalized) {
            MP4.init();
        }
        var movie = MP4.moov(tracks, duration, timescale);
        var result = new Uint8Array(MP4.FTYP.byteLength + movie.byteLength);
        result.set(MP4.FTYP);
        result.set(movie, MP4.FTYP.byteLength);
        return result;
    };
    MP4.fragmentSegment = function (sn, baseMediaDecodeTime, track, payload) {
        var moof = MP4.moof(sn, baseMediaDecodeTime, track);
        var mdat = MP4.mdat(payload);
        var result = new Uint8Array(MP4.STYP.byteLength + moof.byteLength + mdat.byteLength);
        result.set(MP4.STYP);
        result.set(moof, MP4.STYP.byteLength);
        result.set(mdat, MP4.STYP.byteLength + moof.byteLength);
        return result;
    };
    return MP4;
}());
MP4.types = {};
MP4.initalized = false;
exports.default = MP4;
