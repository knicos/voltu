/**
 * VideoPlayer for our stream
 *  
 */

function VideoPlayer(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.status_cb = null;
    this.error_cb = null;
    this.ratio = null;
    this.filters = false;
    this._reset()
}

VideoPlayer.prototype._reset = function() {
    this.start = null;
    this.frames = 0;
    this.image_data = null;
    this.running = false;
    this.pending_image_data = null;
}


/** @expose */
VideoPlayer.prototype.set_status_callback = function(callback) {
    this.status_cb = callback;
};

VideoPlayer.prototype._set_status = function() {
    if (this.status_cb) {
        this.status_cb.apply(this.status_cb, arguments);
    }
};

/** @expose */
VideoPlayer.prototype.set_error_callback = function(callback) {
    this.error_cb = callback;
};

VideoPlayer.prototype._set_error = function(error, message) {
    if (this.error_cb) {
        this.error_cb(error, message);
    }
};

VideoPlayer.prototype._display_image = function(image) {
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




VideoPlayer.prototype._handle_onload = function(peer, decodedURI, uri) {
    var that = this;
    this._set_status("initializing");

    var decoder = new libde265.Decoder();
    decoder.set_image_callback(function(image) {
        that._display_image(image);
        image.free();
    });
    var ratio = null;
    var filters = false;
    

    var decode = function(pckg) {
        if (!that.running) { return; }
        console.log("DECODE FUNKKARI ALKU")
        var err;
        if (pckg == null) { return; }
        else{

            try {
                var tmp = pckg
                err = decoder.push_data(tmp);
            } catch(err) {
                console.log(err);
                err = decoder.flush();
                return;
            }
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
                console.log("DE265_ERROR_WAITING_FOR_INPUT_DATA");
                setTimeout(decode(null), 0);
                return;

            default:
                if (!libde265.de265_isOK(err)) {
                    that._set_error(err, libde265.de265_get_error_text(err));
                    return;
                }
            }

            if (decoder.has_more()) {
                console.log("has more");
                setTimeout(decode(null), 0);
                return;
            }

            decoder.free();
            that.stop();
        });
        console.log("DECODE FUNKKARIN LOPPU")
    }


    peer.bind(decodedURI, (latency, streampckg, pckg) => {
        console.log(pckg[0])
        if(pckg[0] === 0){
            decode([pckg[5]]);
        };
    })
    // Start the transaction
    peer.send("get_stream", (uri, 10, 0, uri));
};

/** @expose */
VideoPlayer.prototype.playback = function(peer, decodedURI, uri) {
    this._reset();

    console.log(peer);
    console.log(uri)
    this._handle_onload(peer, decodedURI, uri)
    // var that = this;

    // peer.sock.onopen = function() {
    //     console.log("Stream open");
    //     that._handle_onload(ws);
    // };

    // ws.onclose = function() { console.log("Connection closed."); }
    this._set_status("loading");
    this.running = true;
    console.log("piippiip")
};

/** @expose */
VideoPlayer.prototype.stop = function() {
    this._set_status("stopped");
    this._reset();
};

/** @expose */
VideoPlayer.prototype.set_framerate_ratio = function(ratio) {
    this.ratio = ratio;
};

/** @expose */
VideoPlayer.prototype.disable_filters = function(disable) {
    this.filters = disable;
};

module.exports = VideoPlayer;