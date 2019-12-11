"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var logger;
var errorLogger;
function setLogger(log, error) {
    logger = log;
    errorLogger = error != null ? error : log;
}
exports.setLogger = setLogger;
function isEnable() {
    return logger != null;
}
exports.isEnable = isEnable;
function log(message) {
    var optionalParams = [];
    for (var _i = 1; _i < arguments.length; _i++) {
        optionalParams[_i - 1] = arguments[_i];
    }
    if (logger) {
        logger.apply(void 0, [message].concat(optionalParams));
    }
}
exports.log = log;
function error(message) {
    var optionalParams = [];
    for (var _i = 1; _i < arguments.length; _i++) {
        optionalParams[_i - 1] = arguments[_i];
    }
    if (errorLogger) {
        errorLogger.apply(void 0, [message].concat(optionalParams));
    }
}
exports.error = error;
