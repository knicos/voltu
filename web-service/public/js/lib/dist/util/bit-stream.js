"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var BitStream = (function () {
    function BitStream(data) {
        this.data = data;
        this.index = 0;
        this.bitLength = data.byteLength * 8;
    }
    Object.defineProperty(BitStream.prototype, "bitsAvailable", {
        get: function () {
            return this.bitLength - this.index;
        },
        enumerable: true,
        configurable: true
    });
    BitStream.prototype.skipBits = function (size) {
        if (this.bitsAvailable < size) {
            throw new Error('no bytes available');
        }
        this.index += size;
    };
    BitStream.prototype.readBits = function (size) {
        var result = this.getBits(size, this.index);
        return result;
    };
    BitStream.prototype.getBits = function (size, offsetBits, moveIndex) {
        if (moveIndex === void 0) { moveIndex = true; }
        if (this.bitsAvailable < size) {
            throw new Error('no bytes available');
        }
        var offset = offsetBits % 8;
        var byte = this.data[(offsetBits / 8) | 0] & (0xff >>> offset);
        var bits = 8 - offset;
        if (bits >= size) {
            if (moveIndex) {
                this.index += size;
            }
            return byte >> (bits - size);
        }
        else {
            if (moveIndex) {
                this.index += bits;
            }
            var nextSize = size - bits;
            return (byte << nextSize) | this.getBits(nextSize, offsetBits + bits, moveIndex);
        }
    };
    BitStream.prototype.skipLZ = function () {
        var leadingZeroCount;
        for (leadingZeroCount = 0; leadingZeroCount < this.bitLength - this.index; ++leadingZeroCount) {
            if (0 !== this.getBits(1, this.index + leadingZeroCount, false)) {
                this.index += leadingZeroCount;
                return leadingZeroCount;
            }
        }
        return leadingZeroCount;
    };
    BitStream.prototype.skipUEG = function () {
        this.skipBits(1 + this.skipLZ());
    };
    BitStream.prototype.skipEG = function () {
        this.skipBits(1 + this.skipLZ());
    };
    BitStream.prototype.readUEG = function () {
        var prefix = this.skipLZ();
        return this.readBits(prefix + 1) - 1;
    };
    BitStream.prototype.readEG = function () {
        var value = this.readUEG();
        if (0x01 & value) {
            return (1 + value) >>> 1;
        }
        else {
            return -1 * (value >>> 1);
        }
    };
    BitStream.prototype.readBoolean = function () {
        return 1 === this.readBits(1);
    };
    BitStream.prototype.readUByte = function () {
        return this.readBits(8);
    };
    BitStream.prototype.readUShort = function () {
        return this.readBits(16);
    };
    BitStream.prototype.readUInt = function () {
        return this.readBits(32);
    };
    return BitStream;
}());
exports.default = BitStream;
