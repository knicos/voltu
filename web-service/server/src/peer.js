const msgpack = require('msgpack5')()
  , encode  = msgpack.encode
  , decode  = msgpack.decode;
const uuidv4 = require('uuid/v4')
const uuidParser = require('./utils/uuidParser')

const kConnecting = 1;
const kConnected = 2;
const kDisconnected = 3;

// Generate a unique id for this webservice
let cpp_my_uuid = uuidv4();
console.log(cpp_my_uuid)
let my_uuid = uuidParser.parse(cpp_my_uuid)
my_uuid = new Uint8Array(my_uuid);
// my_uuid[0] = 44;
// console.log(my_uuid)
my_uuid = Buffer.from(my_uuid);

const kMagic = 0x0009340053640912;
const kVersion = 0;


/**
 * Wrap a web socket with a MsgPack RCP protocol that works with our C++ version.
 * @param {websocket} ws Websocket object
 */
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

	let message = (raw) => {
		// console.log(raw)
		//Gets right data for client
		if(this.sock.on === undefined){
			raw = raw.data;
		}
		let msg = decode(raw);
		// console.log('MSG', msg)
		if (this.status == kConnecting) {
			if (msg[1] != "__handshake__") {
				console.log("Bad handshake");
				this.close();
			}
		}
		if (msg[0] == 0) {
			// console.log("MSG...", msg[2]);
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
	}

	let close = () => {
		this.status = kDisconnected;
		this._notify("disconnect", this);
	}

	let error = () => {
		console.error("Socket error");
		this.sock.close();
		this.status = kDisconnected;
	}

	//if undefined, peer is being used by client
	if(this.sock.on === undefined){
		this.sock.onmessage = message;
		this.sock.onclose = close;
		this.sock.onopen = (event) => {
			this.send("__handshake__", kMagic, kVersion, [my_uuid]);
		}
	//else peer is being used by server
	}else{
		this.sock.on("message", message);
		this.sock.on("close", close);
		this.sock.on("error", error);
	}

	this.bind("__handshake__", (magic, version, id) => {
		if (magic == kMagic) {
			console.log("Handshake received");
			this.status = kConnected;
			this.id = id.buffer;
			this.string_id  = id.toString('hex');
			this._notify("connect", this);
			// if(this.sock.on === undefined){
			// 	this.send("__handshake__", kMagic, kVersion, [my_uuid]);
			// }
		} else {
			console.log("Magic does not match");
			this.close();
		}
	});
	console.log("MY_UUID", my_uuid)
	this.send("__handshake__", kMagic, kVersion, [my_uuid]);
}		


Peer.uuid = my_uuid;

/**
 * @private
 */
Peer.prototype._dispatchNotification = function(name, args) {
	if (this.bindings.hasOwnProperty(name)) {
		//console.log("Notification for: ", name);
		this.bindings[name].apply(this, args);
	} else {
		console.log("Missing handler for: ", name);
	}
}

/**
 * @private
 */
Peer.prototype._dispatchCall = function(name, id, args) {
	console.log("DISPATCHCALL", name, id, args)
	if (this.bindings.hasOwnProperty(name)) {
		//console.log("Call for:", name, id);

		try {
			let res = this.bindings[name].apply(this, args);
			this.sock.send(encode([1,id,name,res]));
		} catch(e) {
			console.error("Could to dispatch or return call", e);
			this.close();
		}
	} else if (this.proxies.hasOwnProperty(name)) {
		//console.log("Proxy for:", name, id);
		args.unshift((res) => {
			try {
				this.sock.send(encode([1,id,name,res]));
			} catch(e) {
				console.log("ERROR")
				this.close();
			}
		});
		this.proxies[name].apply(this, args);
	} else {
		console.log("Missing handler for: ", name);
	}
}

/**
 * @private
 */
Peer.prototype._dispatchResponse = function(id, res) {
	if (this.callbacks.hasOwnProperty(id)) {
		this.callbacks[id].call(this, res);
		delete this.callbacks[id];
	} else {
		console.log("Missing callback");
	}
}

/**
 * Register an RPC handler that will be called from a remote machine. Remotely
 * passed arguments are provided to the given function as normal arguments, and
 * if the function returns a value, it will be returned over the network also.
 * 
 * @param {string} name The name of the function
 * @param {function} f A function or lambda to be callable remotely
 */
Peer.prototype.bind = function(name, f) {
	if (this.bindings.hasOwnProperty(name)) {
		//console.error("Duplicate bind to same procedure");
		this.bindings[name] = f;
	} else {
		this.bindings[name] = f;
	}
}

/**
 * Allow an RPC call to pass through to another machine with minimal local
 * processing.
 */
Peer.prototype.proxy = function(name, f) {
	if (this.proxies.hasOwnProperty(name)) {
		//console.error("Duplicate proxy to same procedure");
		this.proxies[name] = f;
	} else {
		this.proxies[name] = f;
	}
}

/**
 * Call a procedure on a remote machine.
 * 
 * @param {string} name Name of the procedure
 * @param {function} cb Callback to receive return value as argument
 * @param {...} args Any number of arguments to also pass to remote procedure
 */
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

/**
 * Call a remote procedure but with no return value expected.
 * 
 * @param {string} name Name of the procedure
 * @param {...} args Any number of arguments to also pass to remote procedure
 */
Peer.prototype.send = function(name, ...args) {
	try {
		this.sock.send(encode([0, name, args]));
	} catch(e) {
		this.close();
	}
}

/**
 * Closes the socket
 */
Peer.prototype.close = function() {
	if(this.sock.on !== undefined){
		this.sock.close();
	}
	this.status = kDisconnected;
}

/**
 * @private
 */
Peer.prototype._notify = function(evt, ...args) {
	if (this.events.hasOwnProperty(evt)) {
		for (let i=0; i<this.events[evt].length; i++) {
			let f = this.events[evt][i];
			f.apply(this, args);
		}
	}
}

/**
 * Register a callback for socket events. Events include: 'connect',
 * 'disconnect' and 'error'.
 * 
 * @param {string} evt Event name
 * @param {function} f Callback on event
 */
Peer.prototype.on = function(evt, f) {
	if (!this.events.hasOwnProperty(evt)) {
		this.events[evt] = [];
	}
	this.events[evt].push(f);
}

/**
 * Returns a UUID in a string form
 */
Peer.prototype.getUuid = function() {
	const digits = "0123456789abcdef";
	let uuid = "";
	
	return cpp_my_uuid;
}

module.exports = Peer;
