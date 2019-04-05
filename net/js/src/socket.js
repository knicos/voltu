const net = require('net');
const ws = require('ws');
const urijs = require('uri-js');
const binary = require('bops');
const browser = require('detect-browser').detect();
const isbrowser = !browser || browser.name != "node";

function Socket(uri) {
	let t = typeof uri;
	
	this.handlers_ = {
		'open': [],
		'data': [],
		'error': [],
		'close': []
	};
	
	this.handshake_ = false;
	this.lasterr_ = null;
	this.connected_ = false;
	this.valid_ = false;
	this.uuid_ = binary.create(16);
	
	if (!isbrowser) {
		this.buffer_ = new Buffer(0);  // Only in nodejs
	}

	if (t == "string") {
		this._fromURI(uri);
	} else if (t == "object") {
		this._fromObject(uri);
	}
}

Socket.ERROR_BADPROTOCOL = "Bad Protocol";
Socket.ERROR_BADHOST = "Unknown host";
Socket.ERROR_BADHANDSHAKE = "Invalid Handshake";
Socket.ERROR_MALFORMEDURI = "Malformed URI";
Socket.ERROR_TCPINBROWSER = "TCP invalid in browser";
Socket.ERROR_LARGEMESSAGE = "Network message too large";

Socket.prototype.error = function(errno) {
	this.lasterr_ = errno;
	this.dispatch('error', [errno]);
}

Socket.prototype.isValid = function() {
	return this.valid_;
}

/**
 * Construct the correct kind of socket connection from a URI.
 */
Socket.prototype._fromURI = function(uri) {
	let uriobj = urijs.parse(uri);
	this.uri_ = uri;
	this.scheme_ = uriobj.scheme;
	
	// Could not parse uri so report error
	if (uriobj.scheme === undefined || uriobj.host === undefined) {
		this.error(Socket.ERROR_MALFORMEDURI);
		return;
	}
	
	// Websocket protocol
	if (this.scheme_ == "ws") {
		// Detect if in browser or not, choose correct websocket object
		if (typeof WebSocket == "undefined") {
			// Nodejs
			this.socket_ = new ws(uri);
		} else {
			// Browser
			this.socket_ = new WebSocket(uri);
		}
		this._initWebsocket();
	// TCP
	} else if (this.scheme_ == "tcp") {
		if (!isbrowser) {
			this.socket_ = net.connect(uriobj.port, uriobj.host);
			this._initTCPSocket();
		} else {
			this.error(Socket.ERROR_TCPINBROWSER);
		}
	// Unrecognised protocol
	} else {
		this.error(Socket.ERROR_BADPROTOCOL)
	}
}

Socket.prototype._fromObject = function(sock) {
	this.socket_ = sock;
	
	if (typeof WebSocket == "undefined") {
		if (sock instanceof ws) this.scheme_ = "ws";
		else if (sock instanceof net.Socket) this.scheme_ = "tcp";
		else this.scheme_ = null;
	} else {
		if (sock instanceof WebSocket) this.scheme_ = "ws";
		else this.scheme_ = null;
	}
	
	if (this.scheme_ == "ws") this._initWebsocket();
	else if (this.scheme_ == "tcp") this._initTCPSocket();
}

/**
 * Setup correct handlers for a websocket connection.
 */
Socket.prototype._initWebsocket = function() {
	this.valid_ = true;

	let dataHandler = (data) => {
		this.processMessage(data);
	};

	if (this.socket_.addEventHandler) {
		this.socket_.addEventHandler('message', event => {
			dataHandler(event.data);
		});	
	} else {
		this.socket_.on('message', dataHandler);
	}
	this.socket_.on('open', () => {
		//this.connected_ = true;
		//this.dispatch('open', []);
	});
	this.socket_.on('error', (err) => {
		this.connected_ = false;
		this.valid_ = false;
		switch (err.errno) {
		case 'ENOTFOUND'	: this.lasterr_ = Socket.ERROR_BADHOST; break;
		default				: this.lasterr_ = err.errno;
		}
		this.dispatch('error', [this.lasterr_]);
	});
	this.socket_.on('close', () => {
		this.dispatch('close', []);
	});
}

function checkMagic(buffer) {
	if (buffer.length < 8) return false;
	let lo_magic = binary.readUInt32LE(buffer,0);
	let hi_magic = binary.readUInt32LE(buffer,4);
	return (lo_magic == 0x53640912 && hi_magic == 0x10993400)
}

Socket.prototype.processMessage = function(buffer) {
	if (!this.handshake_) {
		// Check handshake
		if (!checkMagic(buffer)) {
			this.close();
			this.error(Socket.ERROR_BADHANDSHAKE);
			return 0;
		}
		
		binary.copy(buffer, this.uuid_, 0, 8, 16);
		let proto_size = binary.readUInt32LE(buffer,24);
		
		this.handshake_ = true;
		this.connected_ = true;
		this.dispatch('open', []);
		
		return 28 + proto_size;
	} else {
		let size = binary.readUInt32LE(buffer,0);
		let service = binary.readUInt32LE(buffer,4);
		
		console.log("Message: " + service + "(size="+size+")");
		
		// Do we have a complete message yet?
		if (size > 1024*1024*100) {
			this.error(Socket.ERROR_LARGEMESSAGE);
			this.close();
			return 0;
		} else if (buffer.length-4 >= size) {
			// Yes, so dispatch
			this.dispatch(service, [size, binary.subarray(buffer,8)]);
			return size+4;
		} else {
			return 0;
		}
	}
}

/**
 * Setup TCP socket handlers and message buffering mechanism.
 */
Socket.prototype._initTCPSocket = function() {
	this.valid_ = true;

	let dataHandler = (data) => {
		this.buffer_ = Buffer.concat([this.buffer_, data]);
		
		while (this.buffer_.length >= 8) {
			let s = this.processMessage(this.buffer_);
			if (s == 0) break;
			this.buffer_ = binary.subarray(this.buffer_,s);
		}
	};
	this.socket_.on('data', dataHandler);
	this.socket_.on('connect', () => {
		//this.connected_ = true;
		//this.dispatch('open', []);
	});
	this.socket_.on('error', (err) => {
		this.connected_ = false;
		this.valid_ = false;
		switch (err.errno) {
		case 'ENOTFOUND'	: this.error(Socket.ERROR_BADHOST); break;
		default				: this.error(err.errno);
		}
	});
	this.socket_.on('close', () => {
		this.dispatch('close', []);
	});
}

Socket.prototype.isConnected = function() {
	return this.connected_;
}

/**
 * Register event handlers.
 */
Socket.prototype.on = function(name, f) {
	if (typeof name == "string") {
		if (this.handlers_.hasOwnProperty(name)) {
			this.handlers_[name].push(f);
		} else {
			console.error("Unrecognised handler: ", name);
		}
		
		if (name == "error" && this.lasterr_ != null) {
			f(this.lasterr_);
		}
	} else if (typeof name == "number") {
		if (this.handlers_[name] === undefined) this.handlers_[name] = [];
		this.handlers_[name].push(f);
	} else {
		console.error("Invalid handler: ", name);
	}
}

Socket.prototype.dispatch = function(h, args) {
	if (this.handlers_.hasOwnProperty(h)) {
		let hs = this.handlers_[h];
		for (var i=0; i<hs.length; i++) {
			hs[i].apply(this, args);
		}
		return true;
	} else {
		return false;
	}
}

Socket.prototype.close = function() {
	if (this.socket_ == null) return;
	
	if (this.scheme_ == "ws") {
		this.socket_.close();
	} else {
		this.socket_.destroy();
	}
	this.socket_ = null;
}

Socket.prototype._socket = function() {
	return this.socket_;
}

Socket.prototype.getURI = function() {
	return this.uri_;
}

Socket.prototype.asyncCall = function(name, cb /*, ...*/) {

}

Socket.prototype.send = function(id /*, ...*/) {
	//this.socket_.write(
}

module.exports = Socket;

