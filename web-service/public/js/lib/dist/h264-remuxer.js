"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var h264_parser_1 = require("./h264-parser");
var debug = require("./util/debug");
var NALU_1 = require("./util/NALU");
var trackId = 1;
var H264Remuxer = (function () {
    function H264Remuxer(fps, framePerFragment, timescale) {
        this.fps = fps;
        this.framePerFragment = framePerFragment;
        this.timescale = timescale;
        this.readyToDecode = false;
        this.totalDTS = 0;
        this.stepDTS = Math.round(this.timescale / this.fps);
        this.frameCount = 0;
        this.seq = 1;
        this.mp4track = {
            id: H264Remuxer.getTrackID(),
            type: 'video',
            len: 0,
            codec: '',
            sps: [],
            pps: [],
            seiBuffering: false,
            width: 0,
            height: 0,
            timescale: timescale,
            duration: timescale,
            samples: [],
            isKeyFrame: true,
        };
        this.unitSamples = [[]];
        this.parser = new h264_parser_1.default(this);
    }
    H264Remuxer.getTrackID = function () {
        return trackId++;
    };
    Object.defineProperty(H264Remuxer.prototype, "seqNum", {
        get: function () {
            return this.seq;
        },
        enumerable: true,
        configurable: true
    });
    H264Remuxer.prototype.remux = function (nalu) {
        if (this.mp4track.seiBuffering && nalu.type() === NALU_1.default.SEI) {
            return this.createNextFrame();
        }
        if (this.parser.parseNAL(nalu)) {
            this.unitSamples[this.unitSamples.length - 1].push(nalu);
            this.mp4track.len += nalu.getSize();
        }
        if (!this.mp4track.seiBuffering && (nalu.type() === NALU_1.default.IDR || nalu.type() === NALU_1.default.NDR)) {
            return this.createNextFrame();
        }
        return;
    };
    H264Remuxer.prototype.createNextFrame = function () {
        if (this.mp4track.len > 0) {
            this.frameCount++;
            if (this.frameCount % this.framePerFragment === 0) {
                var fragment = this.getFragment();
                if (fragment) {
                    var dts = this.totalDTS;
                    this.totalDTS = this.stepDTS * this.frameCount;
                    return [dts, fragment];
                }
                else {
                    debug.log("No mp4 sample data.");
                }
            }
            this.unitSamples.push([]);
        }
        return;
    };
    H264Remuxer.prototype.flush = function () {
        this.seq++;
        this.mp4track.len = 0;
        this.mp4track.samples = [];
        this.mp4track.isKeyFrame = false;
        this.unitSamples = [[]];
    };
    H264Remuxer.prototype.getFragment = function () {
        if (!this.checkReadyToDecode()) {
            return undefined;
        }
        var payload = new Uint8Array(this.mp4track.len);
        this.mp4track.samples = [];
        var offset = 0;
        for (var i = 0, len = this.unitSamples.length; i < len; i++) {
            var units = this.unitSamples[i];
            if (units.length === 0) {
                continue;
            }
            var mp4Sample = {
                size: 0,
                cts: this.stepDTS * i,
            };
            for (var _i = 0, units_1 = units; _i < units_1.length; _i++) {
                var unit = units_1[_i];
                mp4Sample.size += unit.getSize();
                payload.set(unit.getData(), offset);
                offset += unit.getSize();
            }
            this.mp4track.samples.push(mp4Sample);
        }
        if (offset === 0) {
            return undefined;
        }
        return payload;
    };
    H264Remuxer.prototype.checkReadyToDecode = function () {
        if (!this.readyToDecode || this.unitSamples.filter(function (array) { return array.length > 0; }).length === 0) {
            debug.log("Not ready to decode! readyToDecode(" + this.readyToDecode + ") is false or units is empty.");
            return false;
        }
        return true;
    };
    return H264Remuxer;
}());
exports.default = H264Remuxer;
