"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var h264_remuxer_1 = require("./h264-remuxer");
var mp4_generator_1 = require("./mp4-generator");
var debug = require("./util/debug");
var nalu_stream_buffer_1 = require("./util/nalu-stream-buffer");
exports.mimeType = 'video/mp4; codecs="avc1.42E01E"';
var VideoConverter = (function () {
    function VideoConverter(element, fps, fpf) {
        if (fps === void 0) { fps = 60; }
        if (fpf === void 0) { fpf = fps; }
        this.element = element;
        this.fps = fps;
        this.fpf = fpf;
        this.receiveBuffer = new nalu_stream_buffer_1.default();
        this.queue = [];
        if (!MediaSource || !MediaSource.isTypeSupported(exports.mimeType)) {
            throw new Error("Your browser is not supported: " + exports.mimeType);
        }
        this.reset();
    }
    Object.defineProperty(VideoConverter, "errorNotes", {
        get: function () {
            return _a = {},
                _a[MediaError.MEDIA_ERR_ABORTED] = 'fetching process aborted by user',
                _a[MediaError.MEDIA_ERR_NETWORK] = 'error occurred when downloading',
                _a[MediaError.MEDIA_ERR_DECODE] = 'error occurred when decoding',
                _a[MediaError.MEDIA_ERR_SRC_NOT_SUPPORTED] = 'audio/video not supported',
                _a;
            var _a;
        },
        enumerable: true,
        configurable: true
    });
    VideoConverter.prototype.setup = function () {
        var _this = this;
        this.mediaReadyPromise = new Promise(function (resolve, _reject) {
            _this.mediaSource.addEventListener('sourceopen', function () {
                debug.log("Media Source opened.");
                _this.sourceBuffer = _this.mediaSource.addSourceBuffer(exports.mimeType);
                _this.sourceBuffer.addEventListener('updateend', function () {
                    debug.log("  SourceBuffer updateend");
                    debug.log("    sourceBuffer.buffered.length=" + _this.sourceBuffer.buffered.length);
                    for (var i = 0, len = _this.sourceBuffer.buffered.length; i < len; i++) {
                        debug.log("    sourceBuffer.buffered [" + i + "]: " +
                            (_this.sourceBuffer.buffered.start(i) + ", " + _this.sourceBuffer.buffered.end(i)));
                    }
                    debug.log("  mediasource.duration=" + _this.mediaSource.duration);
                    debug.log("  mediasource.readyState=" + _this.mediaSource.readyState);
                    debug.log("  video.duration=" + _this.element.duration);
                    debug.log("    video.buffered.length=" + _this.element.buffered.length);
                    if (debug.isEnable()) {
                        for (var i = 0, len = _this.element.buffered.length; i < len; i++) {
                            debug.log("    video.buffered [" + i + "]: " + _this.element.buffered.start(i) + ", " + _this.element.buffered.end(i));
                        }
                    }
                    debug.log("  video.currentTime=" + _this.element.currentTime);
                    debug.log("  video.readyState=" + _this.element.readyState);
                    var data = _this.queue.shift();
                    if (data) {
                        _this.writeBuffer(data);
                    }
                });
                _this.sourceBuffer.addEventListener('error', function () {
                    debug.error('  SourceBuffer errored!');
                });
                _this.mediaReady = true;
                resolve();
            }, false);
            _this.mediaSource.addEventListener('sourceclose', function () {
                debug.log("Media Source closed.");
                _this.mediaReady = false;
            }, false);
            _this.element.src = URL.createObjectURL(_this.mediaSource);
        });
        return this.mediaReadyPromise;
    };
    VideoConverter.prototype.play = function () {
        var _this = this;
        if (!this.element.paused) {
            return;
        }
        if (this.mediaReady && this.element.readyState >= 2) {
            this.element.play();
        }
        else {
            var handler_1 = function () {
                _this.play();
                _this.element.removeEventListener('canplaythrough', handler_1);
            };
            this.element.addEventListener('canplaythrough', handler_1);
        }
    };
    VideoConverter.prototype.pause = function () {
        if (this.element.paused) {
            return;
        }
        this.element.pause();
    };
    VideoConverter.prototype.reset = function () {
        this.receiveBuffer.clear();
        if (this.mediaSource && this.mediaSource.readyState === 'open') {
            this.mediaSource.duration = 0;
            this.mediaSource.endOfStream();
        }
        this.mediaSource = new MediaSource();
        this.remuxer = new h264_remuxer_1.default(this.fps, this.fpf, this.fps * 60);
        this.mediaReady = false;
        this.mediaReadyPromise = undefined;
        this.queue = [];
        this.isFirstFrame = true;
        this.setup();
    };
    VideoConverter.prototype.appendRawData = function (data, dts) {
        var nalus = this.receiveBuffer.append(data);
        for (var _i = 0, nalus_1 = nalus; _i < nalus_1.length; _i++) {
            var nalu = nalus_1[_i];
            var ret = this.remuxer.remux(nalu);
            if (ret) {
				this.writeFragment(ret[0], ret[1]);  // ret[0]
            }
        }
    };
    VideoConverter.prototype.writeFragment = function (dts, pay) {
        var remuxer = this.remuxer;
        if (remuxer.mp4track.isKeyFrame) {
            this.writeBuffer(mp4_generator_1.default.initSegment([remuxer.mp4track], Infinity, remuxer.timescale));
        }
        if (pay && pay.byteLength) {
            debug.log(" Put fragment: " + remuxer.seqNum + ", frames=" + remuxer.mp4track.samples.length + ", size=" + pay.byteLength);
            var fragment = mp4_generator_1.default.fragmentSegment(remuxer.seqNum, dts, remuxer.mp4track, pay);
            this.writeBuffer(fragment);
            remuxer.flush();
        }
        else {
            debug.error("Nothing payload!");
        }
    };
    VideoConverter.prototype.writeBuffer = function (data) {
        var _this = this;
        if (this.mediaReady) {
            if (this.sourceBuffer.updating) {
                this.queue.push(data);
            }
            else {
                this.doAppend(data);
            }
        }
        else {
            this.queue.push(data);
            if (this.mediaReadyPromise) {
                this.mediaReadyPromise.then(function () {
                    if (!_this.sourceBuffer.updating) {
                        var d = _this.queue.shift();
                        if (d) {
                            _this.writeBuffer(d);
                        }
                    }
                });
                this.mediaReadyPromise = undefined;
            }
        }
    };
    VideoConverter.prototype.doAppend = function (data) {
        var error = this.element.error;
        if (error) {
            debug.error("MSE Error Occured: " + VideoConverter.errorNotes[error.code]);
            this.element.pause();
            if (this.mediaSource.readyState === 'open') {
                this.mediaSource.endOfStream();
            }
        }
        else {
            try {
                this.sourceBuffer.appendBuffer(data);
                debug.log("  appended buffer: size=" + data.byteLength);
            }
            catch (err) {
                debug.error("MSE Error occured while appending buffer. " + err.name + ": " + err.message);
            }
        }
    };
    return VideoConverter;
}());
exports.default = VideoConverter;
