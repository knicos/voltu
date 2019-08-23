const msgpack = require('msgpack5')()
  , encode  = msgpack.encode
  , decode  = msgpack.decode;

const kConnecting = 1;
const kConnected = 2;
const kDisconnected = 3;

let my_uuid = new Uint8Array(16);
my_uuid[0] = 44;
my_uuid = Buffer.from(my_uuid);

const kMagic = 0x0009340053640912;
const kVersion = 0;

function Peer(ws) {
	this.sock = ws;
	this.status = kConnecting;
	this.id = null;
	this.string_id = "";
	this.bindings = {};
	this.proxies = {};
	this.events = {};
	this.callbacks = {};
	this.cbid = 0;

	this.uri = "unknown";
	this.name = "unknown";
	this.master = false;

	this.sock.on("message", (raw) => {
		let msg = decode(raw);
		if (this.status == kConnecting) {
			if (msg[1] != "__handshake__") {
				console.log("Bad handshake");
				this.close();
			}
		}
		//console.log("MSG", msg);
		if (msg[0] == 0) {
			// Notification
			if (msg.length == 3) {
				this._dispatchNotification(msg[1], msg[2]);
			// Call
			} else {
				this._dispatchCall(msg[2], msg[1], msg[3]);
			}
		} else if (msg[0] == 1) {
			this._dispatchResponse(msg[1], msg[3]);
		}
	});

	this.sock.on("close", () => {
		this.status = kDisconnected;
		this._notify("disconnect", this);
	});

	this.sock.on("error", () => {
		console.error("Socket error");
		this.sock.close();
		this.status = kDisconnected;
	});

	this.bind("__handshake__", (magic, version, id) => {
		if (magic == kMagic) {
			console.log("Handshake received");
			this.status = kConnected;
			this.id = id.buffer;
			this.string_id  = id.toString('hex');
			this._notify("connect", this);
		} else {
			console.log("Magic does not match");
			this.close();
		}
	});

	this.send("__handshake__", kMagic, kVersion, [my_uuid]);
}

Peer.uuid = my_uuid;

Peer.prototype._dispatchNotification = function(name, args) {
	if (this.bindings.hasOwnProperty(name)) {
		console.log("Notification for: ", name);
		this.bindings[name].apply(this, args);
	} else {
		console.log("Missing handler for: ", name);
	}
}

Peer.prototype._dispatchCall = function(name, id, args) {
	if (this.bindings.hasOwnProperty(name)) {
		console.log("Call for:", name, id);

		try {
			let res = this.bindings[name].apply(this, args);
			this.sock.send(encode([1,id,name,res]));
		} catch(e) {
			console.error("Could to dispatch or return call");
			this.close();
		}
	} else if (this.proxies.hasOwnProperty(name)) {
		console.log("Proxy for:", name, id);
		args.unshift((res) => {
			try {
				this.sock.send(encode([1,id,name,res]));
			} catch(e) {
				this.close();
			}
		});
		this.proxies[name].apply(this, args);
	} else {
		console.log("Missing handler for: ", name);
	}
}

Peer.prototype._dispatchResponse = function(id, res) {
	if (this.callbacks.hasOwnProperty(id)) {
		this.callbacks[id].call(this, res);
		delete this.callbacks[id];
	} else {
		console.log("Missing callback");
	}
}

Peer.prototype.bind = function(name, f) {
	if (this.bindings.hasOwnProperty(name)) {
		//console.error("Duplicate bind to same procedure");
		this.bindings[name] = f;
	} else {
		this.bindings[name] = f;
	}
}

Peer.prototype.proxy = function(name, f) {
	if (this.proxies.hasOwnProperty(name)) {
		//console.error("Duplicate proxy to same procedure");
		this.proxies[name] = f;
	} else {
		this.proxies[name] = f;
	}
}

Peer.prototype.rpc = function(name, cb, ...args) {
	let id = this.cbid++;
	this.callbacks[id] = cb;

	try {
		this.sock.send(encode([0, id, name, args]));
	} catch(e) {
		this.close();
	}
}

Peer.prototype.sendB = function(name, args) {
	try {
		this.sock.send(encode([0, name, args]));
	} catch(e) {
		this.close();
	}
}

Peer.prototype.send = function(name, ...args) {
	try {
		this.sock.send(encode([0, name, args]));
	} catch(e) {
		this.close();
	}
}

Peer.prototype.close = function() {
	this.sock.close();
	this.status = kDisconnected;
}

Peer.prototype._notify = function(evt, ...args) {
	if (this.events.hasOwnProperty(evt)) {
		for (let i=0; i<this.events[evt].length; i++) {
			let f = this.events[evt][i];
			f.apply(this, args);
		}
	}
}

Peer.prototype.on = function(evt, f) {
	if (!this.events.hasOwnProperty(evt)) {
		this.events[evt] = [];
	}
	this.events[evt].push(f);
}

module.exports = Peer;
