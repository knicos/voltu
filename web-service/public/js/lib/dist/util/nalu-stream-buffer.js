"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var NALU_1 = require("./NALU");
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
                result.push(new NALU_1.default(this.buffer.subarray(4)));
                buffer = Uint8Array.from(value);
            }
        }
        if (buffer == null) {
            buffer = this.mergeBuffer(value);
        }
        var lastIndex = 0;
        var f = nextNalHeader(buffer);
        for (var index = f(); index != null; index = f()) {
            result.push(new NALU_1.default(buffer.subarray(lastIndex + 4, index)));
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
exports.default = VideoStreamBuffer;
