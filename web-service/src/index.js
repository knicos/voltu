const express = require('express');
const app = express();
const expressWs = require('express-ws')(app);
const Peer = require('./peer.js');

// ---- INDEXES ----------------------------------------------------------------

let peer_by_id = {};
//let uri_to_peer = {};
let peer_uris = {};

let uri_data = {};

/**
 * A client stream request object. Each source maintains a list of clients who
 * are wanting frames from that source. Clients can only request N frames at a
 * time, after that if no new request is received then the client is removed.
 * 
 * @param {Peer} peer Peer websocket wrapper
 * @param {number} N Number of frames requested
 * @param {number} rate Bitrate index requested
 * @param {string} dest URI destination
 */
function RGBDClient(peer, N, rate, dest) {
	this.peer = peer;
	this.txmax = N*16;  // 16 is for 16 blocks per frame... this will change in the near future
	this.rate = rate;
	this.dest = dest;
	this.txcount = 0;
}

/**
 * Actually send a frame over network to the client.
 */
RGBDClient.prototype.push = function(uri, latency, spacket, packet) {
	this.peer.send(uri, latency, spacket, packet);
	this.txcount++;
}

/**
 * A video stream. Each peer provides a list of these streams. Each stream can
 * receive frames from the source and forward those frames to any clients.
 * Therefore each of these stream objects maintains a list of clients and
 * loops over them whenever a new frame is received.
 * 
 * @param {string} uri Address of the stream
 * @param {Peer} peer Origin of the stream
 */
function RGBDStream(uri, peer) {
	this.uri = uri;
	this.peer = peer;
	this.title = "";
	this.rgb = null;		// TODO: No longer works as an image
	this.depth = null;		// TODO: No longer works as an image
	this.pose = null;
	this.clients = [];
	this.rxcount = 10;
	this.rxmax = 10;

	// Add RPC handler to receive frames from the source
	peer.bind(uri, (latency, spacket, packet) => {
		// Forward frames to all clients
		this.pushFrames(latency, spacket, packet);
		this.rxcount++;
		if (this.rxcount >= this.rxmax && this.clients.length > 0) {
			this.subscribe();
		}
	});

	/*peer.bind(uri, (frame, ttime, chunk, rgb, depth) => {
		// Forward frames to all clients
		this.pushFrames(frame, ttime, chunk, rgb, depth);
		this.rxcount++;
		if (this.rxcount >= this.rxmax && this.clients.length > 0) {
			this.subscribe();
		}
	});*/
}

RGBDStream.prototype.addClient = function(peer, N, rate, dest) {
	// TODO(Nick) Verify that it isn't already in list...
	for (let i=0; i<this.clients.length; i++) {
		if (this.clients[i].peer.string_id == peer.string_id) return;
	}

	this.clients.push(new RGBDClient(peer, N, rate, dest));

	if (this.rxcount >= this.rxmax) {
		this.subscribe();
	}
}

RGBDStream.prototype.subscribe = function() {
	this.rxcount = 0;
	this.rxmax = 10;
	//console.log("Subscribe to ", this.uri);
	// TODO: Don't hard code 9 here, instead use 9 for thumbnails and 0 for
	// the video...
	this.peer.send("get_stream", this.uri, 10, 9, [Peer.uuid], this.uri);
}

RGBDStream.prototype.pushFrames = function(latency, spacket, packet) {
	if (spacket[1] & 0x1) this.depth = packet[4];
	else this.rgb = packet[4];

	console.log("Frame = ", packet[0], packet[1]);

	for (let i=0; i < this.clients.length; i++) {
		this.clients[i].push(this.uri, latency, spacket, packet);
	}

	let i=0;
	while (i < this.clients.length) {
		if (this.clients[i].txcount >= this.clients[i].txmax) {
			console.log("remove client");
			this.clients.splice(i, 1);
		} else i++;
	}
}

// ---- PROTOCOL ---------------------------------------------------------------

app.get('/', (req, res) => {
	res.end();
});

app.get('/streams', (req, res) => {
	res.json(Object.keys(uri_data));
});

app.get('/stream/rgb', (req, res) => {
	let uri = req.query.uri;
	if (uri_data.hasOwnProperty(uri)) {
		res.writeHead(200, {'Content-Type': 'image/jpeg'});
    	res.end(uri_data[uri].rgb);
	}
	res.end();
});

app.get('/stream/depth', (req, res) => {
	let uri = req.query.uri;
	if (uri_data.hasOwnProperty(uri)) {
		res.writeHead(200, {'Content-Type': 'image/png'});
    	res.end(uri_data[uri].depth);
	}
	res.end();
});

//app.get('/stream', (req, res))

function checkStreams(peer) {
	if (!peer.master) {
		peer.rpc("list_streams", (streams) => {
			console.log("STREAMS", streams);
			for (let i=0; i<streams.length; i++) {
				//uri_to_peer[streams[i]] = peer;
				peer_uris[peer.string_id].push(streams[i]);

				uri_data[streams[i]] = new RGBDStream(streams[i], peer);
			}
		});
	}
}

function broadcastExcept(exc, name, ...args) {
	for (let p in peer_by_id) {
		let peer = peer_by_id[p];
		if (peer === exc) continue;
		peer.sendB(name, args);
	}
}

app.ws('/', (ws, req) => {
	console.log("New web socket request");

	let p = new Peer(ws);

	p.on("connect", (peer) => {
		console.log("Node connected...", peer.string_id);
		peer_uris[peer.string_id] = [];
		peer_by_id[peer.string_id] = peer;

		peer.rpc("node_details", (details) => {
			let obj = JSON.parse(details[0]);

			peer.uri = obj.id;
			peer.name = obj.title;
			peer.master = (obj.kind == "master");
			console.log("Peer name = ", peer.name);
			console.log("Details: ", details);

			checkStreams(peer);
		});
	});

	p.on("disconnect", (peer) => {
		console.log("DISCONNECT");
		// Remove all peer details and streams....

		let puris = peer_uris[peer.string_id];
		if (puris) {
			for (let i=0; i<puris.length; i++) {
				console.log("Removing stream: ", puris[i]);
				//delete uri_to_peer[puris[i]];
				delete uri_data[puris[i]];
			}
			delete peer_uris[peer.string_id];
		}
		if (peer_by_id.hasOwnProperty(peer.string_id)) delete peer_by_id[peer.string_id];
	});

	p.bind("new_peer", (id) => {
		checkStreams(p);
	});

	// Used to sync clocks
	p.bind("__ping__", () => {
		return Date.now();
	});

	p.bind("node_details", () => {
		return ['{"title": "FTL Web-Service", "id": "0", "kind": "master"}'];
	});

	p.bind("list_streams", () => {
		return Object.keys(uri_data);
	});

	p.bind("find_stream", (uri) => {
		if (uri_data.hasOwnProperty(uri)) {
			console.log("Stream found: ", uri);
			return [Peer.uuid];
		} else {
			console.log("Stream not found: ", uri)
			return null; // or []??
		}
	});

	// Requests camera calibration information
	p.proxy("source_details", (cb, uri, chan) => {
		let peer = uri_data[uri].peer;
		if (peer) {
			peer.rpc("source_details", cb, uri, chan);
		}
	});

	// Get the current position of a camera
	p.proxy("get_pose", (cb, uri) => {
		//console.log("SET POSE");
		let peer = uri_data[uri].peer;
		if (peer) {
			peer.rpc("get_pose", cb, uri);
		}
	});

	// Change the position of a camera
	p.bind("set_pose", (uri, vec) => {
		let peer = uri_data[uri].peer;
		if (peer) {
			uri_data[uri].pose = vec;
			peer.send("set_pose", uri, vec);
		}
	});

	// Request from frames from a source
	p.bind("get_stream", (uri, N, rate, pid, dest) => {
		let peer = uri_data[uri].peer;
		if (peer) {
			uri_data[uri].addClient(p, N, rate, dest);
			//peer.send("get_stream", uri, N, rate, [Peer.uuid], dest);
		}
	});

	// Register a new stream
	p.bind("add_stream", (uri) => {
		console.log("Adding stream: ", uri);
		//uri_to_peer[streams[i]] = peer;
		peer_uris[p.string_id].push(uri);

		uri_data[uri] = new RGBDStream(uri, p);

		broadcastExcept(p, "add_stream", uri);
	});
});

console.log("Listening or port 8080");
app.listen(8080);

