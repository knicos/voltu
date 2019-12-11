"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var NALU = (function () {
    function NALU(data) {
        this.data = data;
        this.nri = (data[0] & 0x60) >> 5;
        this.ntype = data[0] & 0x1f;
    }
    Object.defineProperty(NALU, "NDR", {
        get: function () { return 1; },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(NALU, "IDR", {
        get: function () { return 5; },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(NALU, "SEI", {
        get: function () { return 6; },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(NALU, "SPS", {
        get: function () { return 7; },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(NALU, "PPS", {
        get: function () { return 8; },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(NALU, "TYPES", {
        get: function () {
            return _a = {},
                _a[NALU.IDR] = 'IDR',
                _a[NALU.SEI] = 'SEI',
                _a[NALU.SPS] = 'SPS',
                _a[NALU.PPS] = 'PPS',
                _a[NALU.NDR] = 'NDR',
                _a;
            var _a;
        },
        enumerable: true,
        configurable: true
    });
    NALU.type = function (nalu) {
        if (nalu.ntype in NALU.TYPES) {
            return NALU.TYPES[nalu.ntype];
        }
        else {
            return 'UNKNOWN';
        }
    };
    NALU.prototype.type = function () {
        return this.ntype;
    };
    NALU.prototype.isKeyframe = function () {
        return this.ntype === NALU.IDR;
    };
    NALU.prototype.getSize = function () {
        return 4 + this.data.byteLength;
    };
    NALU.prototype.getData = function () {
        var result = new Uint8Array(this.getSize());
        var view = new DataView(result.buffer);
        view.setUint32(0, this.getSize() - 4);
        result.set(this.data, 4);
        return result;
    };
    return NALU;
}());
exports.default = NALU;
