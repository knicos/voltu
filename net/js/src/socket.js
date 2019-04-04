const net = require('net');
const ws = require('ws');
const urijs = require('uri-js');
const binary = require('bops');

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
	this.buffer_ = new Buffer(0);

	if (t == "string") {
		this._fromURI(uri);
	} else if (t == "object") {
		this._fromObject(uri);
	}
}

Socket.ERROR_BADPROTOCOL = 0x01;
Socket.ERROR_BADHOST = 0x02;
Socket.ERROR_BADHANDSHAKE = 0x03;
Socket.ERROR_MALFORMEDURI = 0x04;

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
		this.lasterr_ = Socket.ERROR_MALFORMEDURI;
		this.dispatch('error', [this.lasterr_]);
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
		this.socket_ = net.connect(uriobj.port, uriobj.host);
		this._initTCPSocket();
	// Unrecognised protocol
	} else {
		this.lasterr_ = Socket.ERROR_BADPROTOCOL;
		this.dispatch('error', [this.lasterr_]);
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
		let size = binary.readUInt32LE(data, 0);
		let service = binary.readUInt32LE(data, 4);
		
		console.log("Message", service);
		if (this.handlers_.hasOwnProperty(service)) {
			this.handlers_[service](binary.subarray(data, 8));
		} else {
			console.error("No handler for service "+service);
		}
	};

	if (this.socket_.addEventHandler) {
		this.socket_.addEventHandler('message', event => {
			dataHandler(event.data);
		});	
	} else {
		this.socket_.on('message', dataHandler);
	}
	this.socket_.on('open', () => {
		this.connected_ = true;
		this.dispatch('open', []);
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

Socket.prototype._initTCPSocket = function() {
	this.valid_ = true;

	let dataHandler = (data) => {
		console.log('Received: ' + data);
		this.buffer_ = Buffer.concat([this.buffer_, data]);
		
		if (this.buffer_.length >= 8) {
			let size = binary.readUInt32LE(this.buffer_,0);
			let service = binary.readUInt32LE(this.buffer_,4);
			
			console.log("Message: " + service);
			
			// Do we have a complete message yet?
			if (size > 1024*1024*100) {
				this.dispatch('error', ["invalid message size"]);
				console.log("Message too big");
			} else if (this.buffer_.length-4 >= size) {
				// Yes, so dispatch
				console.log("Complete message found");
				this.dispatch(service, [size, binary.subarray(this.buffer_,8)]);
			} else {
				console.log("Incomplete message");
			}
		}
	};
	this.socket_.on('data', dataHandler);
	this.socket_.on('connect', () => {
		this.connected_ = true;
		this.dispatch('open', []);
	});
	this.socket_.on('error', (err) => {
		this.connected_ = false;
		this.valid_ = false;
		console.log("Sock error: ", err);
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

