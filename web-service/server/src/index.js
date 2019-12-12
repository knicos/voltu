const express = require('express');
const app = express();
const expressWs = require('express-ws')(app);
const Peer = require('./peer.js');
const mongoose = require('mongoose')
const config = require('./utils/config')
const User = require('./models/users')
const Configs = require('./models/generic')
const bodyParser = require('body-parser')
const Url = require('url-parse')

// ---- INDEXES ----------------------------------------------------------------
app.use(express.static(__dirname + '/../../public'));
app.use(bodyParser.json())

// //CONNECTS THE APP TO MONGODB
// mongoose.connect(config.MONGODB_URI, { useNewUrlParser: true, useUnifiedTopology: true })
// 	.then(() => {
// 		console.log('Connected to MongoDB');
// 	})
// 	.catch((err) => {
// 		console.log(err);
// 	})

let peer_by_id = {};

let uri_to_peer = {};

let peer_uris = {};

let uri_data = {};

let peer_data = [];

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
	this.peer.send("get_stream", this.uri, 10, 0, [Peer.uuid], this.uri);
}

RGBDStream.prototype.pushFrames = function(latency, spacket, packet) {
	//Checks that the type is jpg
	if (packet[0] === 0){
		if (spacket[3] > 0) this.depth = packet[5];
		else this.rgb = packet[5];
	}

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


/**
 * A list that has Object.keys(uri_data) values and also the image that is 
 * binded to that 
 */
app.get('/stream/rgb', (req, res) => {
	let uri = req.query.uri;
	if (uri_data.hasOwnProperty(uri)) {
		uri_data[uri].peer.send("get_stream", uri, 3, 0, [Peer.uuid], uri);
		res.writeHead(200, {'Content-Type': 'image/jpeg'});
		res.end(uri_data[uri].rgb);
	}
	res.end();
});


app.get('/stream/depth', (req, res) => {
	let uri = req.query.uri;
	const parsedURI = stringSplitter(uri)
	if (uri_data.hasOwnProperty(parsedURI)) {
		res.writeHead(200, {'Content-Type': 'image/png'});
    	res.end(uri_data[parsedURI].depth);
	}
	res.end();
});

app.post('/stream/config', async (req, res) => {
	// const rawData = JSON.parse(req.body);
	const {peerURI, configURI, data, saveToCPP} = req.body;
	const parsedURI = stringSplitter(peerURI)

	if(saveToCPP){
		try{
			let peer = uri_data[parsedURI].peer
			if(peer){
				peer.send("update_cfg", configURI, data)
				return res.status(200).json("Successfully saved configs")
			}
			return res.status(502).json("Something went wrong")
		}catch(e) {
			console.log(e)
		}

	}// else{
	// //Save to MongoDB
	// const savedConfigs = new Configs({
	// 	settingsURI: configURI,
	// 	data
	// });

	// try{
	// 	await savedConfigs.save();
	// 	return res.status(200).json('Your configurations were saved successfully')
	// }catch(err){
	// 	console.log(err)
	// 	return res.status(500).json("Something's wrong I can feel it")
	// }
	// }

})

app.get('/stream/config', async(req, res) => {
	
	//example of uri ftlab.utu.fi/stream/config?uri=ftl://utu.fi#reconstruction_snap10/merge
	const settings = req.query.settings;
	const uri = req.query.uri;
	const parsedURI = stringSplitter(uri)

	// //Checks if DB has data
	// let dbData = await Configs.find({Settings: settings});
	// if(dbData[0].data){
	// 	return res.status(200).json(dbData[0]);
	// }else{
		let peer = uri_data[parsedURI].peer
		if(peer){
			peer.rpc("get_cfg", (response) => {
				if(response){
					return res.status(200).json(response);
				}
			}, settings)
		}
	// }
})


app.get('/stream', (req, res) => {
	//If wanted, this could render new html file dedicated to the actual livestream function
	let uri = req.query.uri;
	res.end();
})


function checkStreams(peer) {
	if (!peer.master) {
		peer.rpc("list_streams", (streams) => {
			console.log("STREAMS", streams);
			for (let i=0; i<streams.length; i++) {
				//uri_to_peer[streams[i]] = peer;
				let parsedURI = stringSplitter(streams[i])
				peer_uris[peer.string_id].push(parsedURI);
				uri_to_peer[parsedURI] = peer;
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
	//console.log('WEBSOCKET', ws)
	
	let p = new Peer(ws);
	peer_data.push(p);

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
		return [`{"title": "FTL Web-Service", "id": "${p.getUuid()}", "kind": "master"}`];
	});

	p.bind("list_streams", () => {
		return Object.keys(uri_data);
	});

	p.bind("find_stream", (uri) => {
		const parsedURI = stringSplitter(uri)
		if (uri_to_peer.hasOwnProperty(parsedURI)) {
			console.log("Stream found: ", uri);
			return [Peer.uuid];
		} else {
			console.log("Stream not found: ", uri)
			return null; // or []??
		}
	});

	// Requests camera calibration information
	p.proxy("source_details", (cb, uri, chan) => {
		const parsedURI = stringSplitter(uri);
		if(uri_to_peer[parsedURI]){
			let peer = uri_to_peer[parsedURI].peer
			if (peer) {
				peer.rpc("source_details", cb, uri, chan);
			}
		}else{
			console.log("Failed to get source details for URI", uri);
			return "{}"
		}
	});

	// Get the current position of a camera
	p.proxy("get_pose", (cb, uri) => {
		//console.log("SET POSE");
		const parsedURI = stringSplitter(uri);
		if(uri_to_peer[parsedURI]){
			let peer = uri_to_peer[parsedURI].peer
			if (peer) {
				peer.rpc("get_pose", cb, uri);
			}
		}else{
			console.log("Failed to get pose for URI", uri);
			return "{}"
		}
	});

	// Change the position of a camera
	p.bind("set_pose", (uri, vec) => {
		const parsedURI = stringSplitter(uri);
		if(uri_to_peer[parsedURI]){
			let peer = uri_to_peer[parsedURI].peer
			if (peer) {
				uri_data[parsedURI].pose = vec;
				peer.send("set_pose", uri, vec);
			}
		}else{
			console.log("Couldn't set pose for URI", uri)
			return "{}";
		}
	});

	// Request from frames from a source
	p.bind("get_stream", (uri, N, rate, /*pid,*/ dest) => {
		console.log(uri)
		const parsedURI = stringSplitter(uri);
		if(uri_data[uri]){
			let peer = uri_data[uri].peer
			console.log(peer)
			if (peer) {
				console.log("GET_STREAM IS WORKING VERY WELL")
				uri_data[uri].addClient(p, N, rate, dest);
			//peer.send("get_stream", uri, N, rate, [Peer.uuid], dest);
			}
		}else{
			console.log("Couldn't get stream for ", uri)
			return "{}";
		}
	});

	/**
	 * Get JSON values for stream configuration
	 */ 
	p.bind("get_cfg", (cb, uri) => {
		const parsedURI = stringSplitter(uri);
		if(uri_to_peer[parsedURI]){
			let peer = uri_to_peer[parsedURI].peer
			if(peer){
				peer.rpc("get_cfg", cb, uri)
			}	
		}else{
			console.log("Config not found", uri)
			return "{}";
		}
	})

	/**
	 * Update certain URIs values
	 */
	 p.bind("update_cfg", (uri, json) => {
		const parsedURI = stringSplitter(uri)
		console.log("URI", uri)
		console.log("JSON", json)
		if(uri_to_peer[parsedURI]){
			let peer = uri_to_peer[parsedURI]
			peer.send("update_cfg", uri, json)
		}else{
			console.log("Failed to update the configuration uri", uri)
			return "{}";
		}
	 })

	// Register a new stream
	p.bind("add_stream", (uri) => {
		const parsedURI = stringSplitter(uri)
		console.log("Adding stream: ", uri);
		//uri_to_peer[streams[i]] = peer;
		peer_uris[p.string_id].push(parsedURI);
		uri_to_peer[parsedURI] = p;
		uri_data[uri] = new RGBDStream(uri, p);

		broadcastExcept(p, "add_stream", uri);
	});
});

/**
 * Returns the first part of the URI
 * e.g. ftl://utu.fi or ftl://something.fi
 * @param {uri} uri 
 */
function stringSplitter(uri) {
	const url = new Url(uri)
	return url.origin;
}

console.log("Listening or port 8080");
app.listen(8080);

