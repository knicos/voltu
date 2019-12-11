"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var bit_stream_1 = require("./util/bit-stream");
var debug = require("./util/debug");
var NALU_1 = require("./util/NALU");
var H264Parser = (function () {
    function H264Parser(remuxer) {
        this.remuxer = remuxer;
        this.track = remuxer.mp4track;
    }
    H264Parser.prototype.parseSEI = function (sei) {
        var messages = H264Parser.readSEI(sei);
        for (var _i = 0, messages_1 = messages; _i < messages_1.length; _i++) {
            var m = messages_1[_i];
            switch (m.type) {
                case 0:
                    this.track.seiBuffering = true;
                    break;
                case 5:
                    return true;
                default:
                    break;
            }
        }
        return false;
    };
    H264Parser.prototype.parseSPS = function (sps) {
        var config = H264Parser.readSPS(sps);
        this.track.width = config.width;
        this.track.height = config.height;
        this.track.sps = [sps];
        this.track.codec = 'avc1.';
        var codecArray = new DataView(sps.buffer, sps.byteOffset + 1, 4);
        for (var i = 0; i < 3; ++i) {
            var h = codecArray.getUint8(i).toString(16);
            if (h.length < 2) {
                h = '0' + h;
            }
            this.track.codec += h;
        }
    };
    H264Parser.prototype.parsePPS = function (pps) {
        this.track.pps = [pps];
    };
    H264Parser.prototype.parseNAL = function (unit) {
        if (!unit) {
            return false;
        }
        var push = false;
        switch (unit.type()) {
            case NALU_1.default.NDR:
            case NALU_1.default.IDR:
                push = true;
                break;
            case NALU_1.default.SEI:
                push = this.parseSEI(unit.getData().subarray(4));
                break;
            case NALU_1.default.SPS:
                if (this.track.sps.length === 0) {
                    this.parseSPS(unit.getData().subarray(4));
                    debug.log(" Found SPS type NALU frame.");
                    if (!this.remuxer.readyToDecode && this.track.pps.length > 0 && this.track.sps.length > 0) {
                        this.remuxer.readyToDecode = true;
                    }
                }
                break;
            case NALU_1.default.PPS:
                if (this.track.pps.length === 0) {
                    this.parsePPS(unit.getData().subarray(4));
                    debug.log(" Found PPS type NALU frame.");
                    if (!this.remuxer.readyToDecode && this.track.pps.length > 0 && this.track.sps.length > 0) {
                        this.remuxer.readyToDecode = true;
                    }
                }
                break;
            default:
                debug.log(" Found Unknown type NALU frame. type=" + unit.type());
                break;
        }
        return push;
    };
    H264Parser.skipScalingList = function (decoder, count) {
        var lastScale = 8;
        var nextScale = 8;
        for (var j = 0; j < count; j++) {
            if (nextScale !== 0) {
                var deltaScale = decoder.readEG();
                nextScale = (lastScale + deltaScale + 256) % 256;
            }
            lastScale = (nextScale === 0) ? lastScale : nextScale;
        }
    };
    H264Parser.readSPS = function (data) {
        var decoder = new bit_stream_1.default(data);
        var frameCropLeftOffset = 0;
        var frameCropRightOffset = 0;
        var frameCropTopOffset = 0;
        var frameCropBottomOffset = 0;
        var sarScale = 1;
        decoder.readUByte();
        var profileIdc = decoder.readUByte();
        decoder.skipBits(5);
        decoder.skipBits(3);
        decoder.skipBits(8);
        decoder.skipUEG();
        if (profileIdc === 100 ||
            profileIdc === 110 ||
            profileIdc === 122 ||
            profileIdc === 244 ||
            profileIdc === 44 ||
            profileIdc === 83 ||
            profileIdc === 86 ||
            profileIdc === 118 ||
            profileIdc === 128) {
            var chromaFormatIdc = decoder.readUEG();
            if (chromaFormatIdc === 3) {
                decoder.skipBits(1);
            }
            decoder.skipUEG();
            decoder.skipUEG();
            decoder.skipBits(1);
            if (decoder.readBoolean()) {
                var scalingListCount = (chromaFormatIdc !== 3) ? 8 : 12;
                for (var i = 0; i < scalingListCount; ++i) {
                    if (decoder.readBoolean()) {
                        if (i < 6) {
                            H264Parser.skipScalingList(decoder, 16);
                        }
                        else {
                            H264Parser.skipScalingList(decoder, 64);
                        }
                    }
                }
            }
        }
        decoder.skipUEG();
        var picOrderCntType = decoder.readUEG();
        if (picOrderCntType === 0) {
            decoder.readUEG();
        }
        else if (picOrderCntType === 1) {
            decoder.skipBits(1);
            decoder.skipEG();
            decoder.skipEG();
            var numRefFramesInPicOrderCntCycle = decoder.readUEG();
            for (var i = 0; i < numRefFramesInPicOrderCntCycle; ++i) {
                decoder.skipEG();
            }
        }
        decoder.skipUEG();
        decoder.skipBits(1);
        var picWidthInMbsMinus1 = decoder.readUEG();
        var picHeightInMapUnitsMinus1 = decoder.readUEG();
        var frameMbsOnlyFlag = decoder.readBits(1);
        if (frameMbsOnlyFlag === 0) {
            decoder.skipBits(1);
        }
        decoder.skipBits(1);
        if (decoder.readBoolean()) {
            frameCropLeftOffset = decoder.readUEG();
            frameCropRightOffset = decoder.readUEG();
            frameCropTopOffset = decoder.readUEG();
            frameCropBottomOffset = decoder.readUEG();
        }
        if (decoder.readBoolean()) {
            if (decoder.readBoolean()) {
                var sarRatio = void 0;
                var aspectRatioIdc = decoder.readUByte();
                switch (aspectRatioIdc) {
                    case 1:
                        sarRatio = [1, 1];
                        break;
                    case 2:
                        sarRatio = [12, 11];
                        break;
                    case 3:
                        sarRatio = [10, 11];
                        break;
                    case 4:
                        sarRatio = [16, 11];
                        break;
                    case 5:
                        sarRatio = [40, 33];
                        break;
                    case 6:
                        sarRatio = [24, 11];
                        break;
                    case 7:
                        sarRatio = [20, 11];
                        break;
                    case 8:
                        sarRatio = [32, 11];
                        break;
                    case 9:
                        sarRatio = [80, 33];
                        break;
                    case 10:
                        sarRatio = [18, 11];
                        break;
                    case 11:
                        sarRatio = [15, 11];
                        break;
                    case 12:
                        sarRatio = [64, 33];
                        break;
                    case 13:
                        sarRatio = [160, 99];
                        break;
                    case 14:
                        sarRatio = [4, 3];
                        break;
                    case 15:
                        sarRatio = [3, 2];
                        break;
                    case 16:
                        sarRatio = [2, 1];
                        break;
                    case 255: {
                        sarRatio = [decoder.readUByte() << 8 | decoder.readUByte(), decoder.readUByte() << 8 | decoder.readUByte()];
                        break;
                    }
                    default: {
                        debug.error("  H264: Unknown aspectRatioIdc=" + aspectRatioIdc);
                    }
                }
                if (sarRatio) {
                    sarScale = sarRatio[0] / sarRatio[1];
                }
            }
            if (decoder.readBoolean()) {
                decoder.skipBits(1);
            }
            if (decoder.readBoolean()) {
                decoder.skipBits(4);
                if (decoder.readBoolean()) {
                    decoder.skipBits(24);
                }
            }
            if (decoder.readBoolean()) {
                decoder.skipUEG();
                decoder.skipUEG();
            }
            if (decoder.readBoolean()) {
                var unitsInTick = decoder.readUInt();
                var timeScale = decoder.readUInt();
                var fixedFrameRate = decoder.readBoolean();
                var frameDuration = timeScale / (2 * unitsInTick);
                debug.log("timescale: " + timeScale + "; unitsInTick: " + unitsInTick + "; " +
                    ("fixedFramerate: " + fixedFrameRate + "; avgFrameDuration: " + frameDuration));
            }
        }
        return {
            width: Math.ceil((((picWidthInMbsMinus1 + 1) * 16) - frameCropLeftOffset * 2 - frameCropRightOffset * 2) * sarScale),
            height: ((2 - frameMbsOnlyFlag) * (picHeightInMapUnitsMinus1 + 1) * 16) -
                ((frameMbsOnlyFlag ? 2 : 4) * (frameCropTopOffset + frameCropBottomOffset)),
        };
    };
    H264Parser.readSEI = function (data) {
        var decoder = new bit_stream_1.default(data);
        decoder.skipBits(8);
        var result = [];
        while (decoder.bitsAvailable > 3 * 8) {
            result.push(this.readSEIMessage(decoder));
        }
        return result;
    };
    H264Parser.readSEIMessage = function (decoder) {
        function get() {
            var result = 0;
            while (true) {
                var value = decoder.readUByte();
                result += value;
                if (value !== 0xff) {
                    break;
                }
            }
            return result;
        }
        var payloadType = get();
        var payloadSize = get();
        return this.readSEIPayload(decoder, payloadType, payloadSize);
    };
    H264Parser.readSEIPayload = function (decoder, type, size) {
        var result;
        switch (type) {
            default:
                result = { type: type };
                decoder.skipBits(size * 8);
        }
        decoder.skipBits(decoder.bitsAvailable % 8);
        return result;
    };
    return H264Parser;
}());
exports.default = H264Parser;
