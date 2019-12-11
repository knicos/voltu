"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var h264_1 = require("./h264");
var mp4_generator_1 = require("./mp4-generator");
var NALU_1 = require("./util/NALU");
exports.mimeType = 'video/mp4; codecs="avc1.42E01E"';
var fps = 30;
var fpf = 6;
var VideoController = (function () {
    function VideoController(element) {
        this.element = element;
        this.receiveBuffer = new VideoStreamBuffer();
        this.queue = [];
        if (!MediaSource || !MediaSource.isTypeSupported(exports.mimeType)) {
            throw new Error("Your browser is not supported: " + exports.mimeType);
        }
        this.reset();
    }
    Object.defineProperty(VideoController, "errorNotes", {
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
    VideoController.prototype.setup = function () {
        var _this = this;
        this.mediaReadyPromise = new Promise(function (resolve, _reject) {
            _this.mediaSource.addEventListener('sourceopen', function () {
                console.log("Media Source opened.");
                _this.sourceBuffer = _this.mediaSource.addSourceBuffer(exports.mimeType);
                _this.sourceBuffer.addEventListener('updateend', function () {
                    console.log("  SourceBuffer updateend");
                    console.log("    sourceBuffer.buffered.length=" + _this.sourceBuffer.buffered.length);
                    for (var i = 0, len = _this.sourceBuffer.buffered.length; i < len; i++) {
                        console.log("    sourceBuffer.buffered [" + i + "]: " + _this.sourceBuffer.buffered.start(i) + ", " + _this.sourceBuffer.buffered.end(i));
                    }
                    console.log("  mediasource.duration=" + _this.mediaSource.duration);
                    console.log("  mediasource.readyState=" + _this.mediaSource.readyState);
                    console.log("  video.duration=" + _this.element.duration);
                    console.log("    video.buffered.length=" + _this.element.buffered.length);
                    for (var i = 0, len = _this.element.buffered.length; i < len; i++) {
                        console.log("    video.buffered [" + i + "]: " + _this.element.buffered.start(i) + ", " + _this.element.buffered.end(i));
                    }
                    console.log("  video.currentTimen=" + _this.element.currentTime);
                    console.log("  video.readyState=" + _this.element.readyState);
                    var data = _this.queue.shift();
                    if (data) {
                        _this.writeBuffer(data);
                    }
                });
                _this.sourceBuffer.addEventListener('error', function () {
                    console.error('  SourceBuffer errored!');
                });
                _this.mediaReady = true;
                resolve();
            }, false);
            _this.mediaSource.addEventListener('sourceclose', function () {
                console.log("Media Source closed.");
                _this.mediaReady = false;
            }, false);
            _this.element.src = URL.createObjectURL(_this.mediaSource);
        });
        return this.mediaReadyPromise;
    };
    VideoController.prototype.play = function () {
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
    VideoController.prototype.pause = function () {
        if (this.element.paused) {
            return;
        }
        this.element.pause();
    };
    VideoController.prototype.reset = function () {
        this.receiveBuffer.clear();
        if (this.mediaSource && this.mediaSource.readyState === 'open') {
            this.mediaSource.duration = 0;
            this.mediaSource.endOfStream();
        }
        this.mediaSource = new MediaSource();
        this.remuxer = new h264_1.H264Remuxer(fps, fpf, fps * 60);
        this.mediaReady = false;
        this.mediaReadyPromise = undefined;
        this.queue = [];
        this.isFirstFrame = true;
    };
    VideoController.prototype.appendRawData = function (data) {
        var nalus = this.receiveBuffer.append(data);
        for (var _i = 0, nalus_1 = nalus; _i < nalus_1.length; _i++) {
            var nalu = nalus_1[_i];
            var ret = this.remuxer.remux(nalu);
            if (ret) {
                this.writeFragment(ret[0], ret[1]);
            }
        }
    };
    VideoController.prototype.writeFragment = function (dts, pay) {
        var remuxer = this.remuxer;
        if (remuxer.mp4track.isKeyFrame) {
            this.writeBuffer(mp4_generator_1.MP4.initSegment([remuxer.mp4track], Infinity, remuxer.timescale));
        }
        if (pay && pay.byteLength) {
            console.log(" Put framgment: " + remuxer.seq + ", frames=" + remuxer.mp4track.samples.length + ", size=" + pay.byteLength);
            var fragment = mp4_generator_1.MP4.fragmentSegment(remuxer.seq, dts, remuxer.mp4track, pay);
            this.writeBuffer(fragment);
            remuxer.flush();
        }
        else {
            console.error("Nothing payload!");
        }
    };
    VideoController.prototype.writeBuffer = function (data) {
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
    VideoController.prototype.doAppend = function (data) {
        var error = this.element.error;
        if (error) {
            console.error("MSE Error Occured: " + VideoController.errorNotes[error.code]);
            this.element.pause();
            if (this.mediaSource.readyState === 'open') {
                this.mediaSource.endOfStream();
            }
        }
        else {
            try {
                this.sourceBuffer.appendBuffer(data);
                console.log("  appended buffer: size=" + data.byteLength);
            }
            catch (err) {
                console.error("MSE Error occured while appending buffer. " + err.name + ": " + err.message);
            }
        }
    };
    return VideoController;
}());
exports.VideoController = VideoController;
var VideoStreamBuffer = (function () {
    function VideoStreamBuffer() {
    }
    VideoStreamBuffer.prototype.clear = function () {
        this.buffer = undefined;
    };
    VideoStreamBuffer.prototype.append = function (value) {
        var nextNalHeader = function (b) {
            var i = 3;
            return function () {
                var count = 0;
                for (; i < b.length; i++) {
                    switch (b[i]) {
                        case 0:
                            count++;
                            break;
                        case 1:
                            if (count === 3) {
                                return i - 3;
                            }
                        default:
                            count = 0;
                    }
                }
                return;
            };
        };
        var result = [];
        var buffer;
        if (this.buffer) {
            if (value[3] === 1 && value[2] === 0 && value[1] === 0 && value[0] === 0) {
                result.push(new NALU_1.NALU(this.buffer.subarray(4)));
                buffer = Uint8Array.from(value);
            }
        }
        if (buffer == null) {
            buffer = this.mergeBuffer(value);
        }
        var index;
        var lastIndex = 0;
        var f = nextNalHeader(buffer);
        while (index = f()) {
            result.push(new NALU_1.NALU(buffer.subarray(lastIndex + 4, index)));
            lastIndex = index;
        }
        this.buffer = buffer.subarray(lastIndex);
        return result;
    };
    VideoStreamBuffer.prototype.mergeBuffer = function (value) {
        if (this.buffer == null) {
            return Uint8Array.from(value);
        }
        else {
            var newBuffer = new Uint8Array(this.buffer.byteLength + value.length);
            if (this.buffer.byteLength > 0) {
                newBuffer.set(this.buffer, 0);
            }
            newBuffer.set(value, this.buffer.byteLength);
            return newBuffer;
        }
    };
    return VideoStreamBuffer;
}());
