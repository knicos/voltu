var ee = require('event-emitter');
const FTLRemux = require('./ftlremux');

function FTLMSE(video) {
	this.video = video;
	this.remux = new FTLRemux();

	this.paused = false;
	this.active = false;

	this.remux.on('data', (data) => {
		if (this.sourceBuffer.updating) {
			this.queue.push(data);
		} else {
			//console.log("Direct append: ", data);

			try {
				this.sourceBuffer.appendBuffer(data);
			} catch (e) {
				console.error("Failed to append buffer");
			}
		}
	});

	// TODO: Generate
	//this.mime = 'video/mp4; codecs="avc1.640028, opus"';
	this.mime = null;
	
	this.mediaSource = new MediaSource();
	//this.element.play();
	this.sourceBuffer = null;

	this.video.addEventListener('pause', (e) => {
		console.log("pause");
		this.active = false;
	});

	this.video.addEventListener('play', (e) => {
		console.log("Play");
		this.active = true;
		this.remux.select(0,0,0);
	});

	this.mediaSource.addEventListener('sourceopen', (e) => {
		console.log("Source Open", e);
		URL.revokeObjectURL(this.video.src);
		console.log(this.mediaSource.readyState);
		this.sourceBuffer = e.target.addSourceBuffer(this.mime);
		//this.sourceBuffer.mode = 'sequence';
		this.active = true;

		this.sourceBuffer.addEventListener('error', (e) => {
			console.error("SourceBuffer: ", e);
			this.active = false;
		});

		this.sourceBuffer.addEventListener('updateend', () => {
			if (this.queue.length > 0 && !this.sourceBuffer.updating) {
				let s = this.queue[0];
				this.queue.shift();
				//console.log("Append", s);

				try {
					this.sourceBuffer.appendBuffer(s);
				} catch(e) {
					console.error("Failed to append buffer");
				}
			}
		});
	});

	this.queue = [];
	//this.video.src = URL.createObjectURL(this.mediaSource);

	this.has_audio = false;
	this.first_ts = 0;
}

ee(FTLMSE.prototype);

FTLMSE.prototype.push = function(spkt, pkt) {
	if (this.first_ts == 0) this.first_ts = spkt[0];

	// Skip first 200ms, use to analyse the stream contents
	if (spkt[0] < this.first_ts + 200) {
		if (spkt[3] == 32 || spkt[3] == 33) this.has_audio = true;
	} else {
		if (!this.mime) {
			if (this.has_audio) {
				console.log("Create video with audio");
				this.mime = 'video/mp4; codecs="avc1.640028, opus"';
				this.remux.has_audio = true;
			} else {
				console.log("Create video without audio");
				this.mime = 'video/mp4; codecs="avc1.640028"';
				this.remux.has_audio = false;
			}
			this.video.src = URL.createObjectURL(this.mediaSource);			
		}
		this.remux.push(spkt,pkt);
	}
}

FTLMSE.prototype.select = function(frameset, source, channel) {
	this.remux.select(frameset, source, channel);
}

module.exports = FTLMSE;
