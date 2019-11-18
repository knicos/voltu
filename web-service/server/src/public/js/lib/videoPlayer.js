/**
 * VideoPlayer for our stream
 *  
 */

let videoPlayer =function(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.status_cb = null;
    this.error_cb = null;
    this.ratio = null;
    this.filters = false;
    this._reset();
}

videoPlayer.prototype._reset = () => {
    this.start = null;
    this.frames = 0;
    this.image_data = null;
    this.running = false;
    this.pending_image_data = null;
}


/** @expose */
videoPlayer.prototype.set_status_callback = function(callback) {
    this.status_cb = callback;
};

videoPlayer.prototype._set_status = function() {
    if (this.status_cb) {
        this.status_cb.apply(this.status_cb, arguments);
    }
};

/** @expose */
videoPlayer.prototype.set_error_callback = function(callback) {
    this.error_cb = callback;
};

videoPlayer.prototype._set_error = function(error, message) {
    if (this.error_cb) {
        this.error_cb(error, message);
    }
};

videoPlayer.prototype._display_image = function(image) {
    if (!this.start) {
        this.start = new Date();
        this._set_status("playing");
    } else {
        this.frames += 1;
        var duration = (new Date()) - this.start;
        if (duration > 1000) {
            this._set_status("fps", this.frames / (duration * 0.001));
        }
    }

    var w = image.get_width();
    var h = image.get_height();
    if (w != this.canvas.width || h != this.canvas.height || !this.image_data) {
        this.canvas.width = w;
        this.canvas.height = h;
        this.image_data = this.ctx.createImageData(w, h);
        var image_data = this.image_data.data;
        for (var i=0; i<w*h; i++) {
            image_data[i*4+3] = 255;
        }
    }

    var that = this;
    image.display(this.image_data, function(display_image_data) {
        if (window.requestAnimationFrame) {
            that.pending_image_data = display_image_data;
            window.requestAnimationFrame(function() {
                if (that.pending_image_data) {
                    that.ctx.putImageData(that.pending_image_data, 0, 0);
                    that.pending_image_data = null;
                }
            });
        } else {
            that.ctx.putImageData(display_image_data, 0, 0);
        }
    });
};

videoPlayer.prototype._handle_onload = function(request, event) {
    var that = this;
    this._set_status("initializing");

    var decoder = new Decoder();
    decoder.set_image_callback(function(image) {
        that._display_image(image);
        image.free();
    });

    var data = request.response;
    var pos = 0;
    var remaining = data.byteLength;
    var ratio = null;
    var filters = false;

    var decode = function() {
        if (!that.running) {
            return;
        }

        var err;
        if (remaining === 0) {
            err = decoder.flush();
        } else {
            var l = 4096;
            if (l > remaining) {
                l = remaining;
            }
            var tmp = new Uint8Array(data, pos, l);
            err = decoder.push_data(tmp);
            pos += l;
            remaining -= l;
        }
        if (!libde265.de265_isOK(err)) {
            that._set_error(err, libde265.de265_get_error_text(err));
            return;
        }

        if (that.ratio !== ratio) {
            decoder.set_framerate_ratio(that.ratio);
            ratio = that.ratio;
        }

        if (that.filters !== filters) {
            decoder.disable_filters(that.filters);
            filters = that.filters;
        }

        decoder.decode(function(err) {
            switch(err) {
            case libde265.DE265_ERROR_WAITING_FOR_INPUT_DATA:
                setTimeout(decode, 0);
                return;

            default:
                if (!libde265.de265_isOK(err)) {
                    that._set_error(err, libde265.de265_get_error_text(err));
                    return;
                }
            }

            if (remaining > 0 || decoder.has_more()) {
                setTimeout(decode, 0);
                return;
            }

            decoder.free();
            that.stop();
        });
    };

    setTimeout(decode, 0);
};

/** @expose */
videoPlayer.prototype.playback = function(url) {
    this._reset();
    var request = new XMLHttpRequest();
    request.open("get", url, true);
    request.responseType = "arraybuffer";
    var that = this;
    request.onload = function(event) {
        that._handle_onload(request, event);
    };
    this._set_status("loading");
    this.running = true;
    request.send();
};

/** @expose */
videoPlayer.prototype.stop = function() {
    this._set_status("stopped");
    this._reset();
};

/** @expose */
videoPlayer.prototype.set_framerate_ratio = function(ratio) {
    this.ratio = ratio;
};

/** @expose */
videoPlayer.prototype.disable_filters = function(disable) {
    this.filters = disable;
};

module.exports = videoPlayer;