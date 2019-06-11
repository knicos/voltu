const express = require('express');
const app = express();
const expressWs = require('express-ws')(app);
const Peer = require('./peer.js');

// ---- INDEXES ----------------------------------------------------------------

let peer_by_id = {};
//let uri_to_peer = {};
let peer_uris = {};

let uri_data = {};

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

app.ws('/', (ws, req) => {
	console.log("New web socket request");

	let p = new Peer(ws);

	p.on("connect", (peer) => {
		peer.rpc("node_details", (details) => {
			let obj = JSON.parse(details[0]);

			peer.uri = obj.id;
			peer.name = obj.title;
			peer.master = (obj.kind == "master");
			console.log("Peer name = ", peer.name);

			peer_uris[peer.string_id] = [];
			peer_by_id[peer.string_id] = peer;

			if (!peer.master) {
				peer.rpc("list_streams", (streams) => {
					console.log("STREAMS", streams);
					for (let i=0; i<streams.length; i++) {
						//uri_to_peer[streams[i]] = peer;
						peer_uris[peer.string_id].push(streams[i]);

						uri_data[streams[i]] = {
							peer: peer,
							title: "",
							rgb: null,
							depth: null,
							pose: null
						};
					}
				});
			}
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

	p.bind("list_streams", () => {
		return Object.keys(uri_data);
	});

	p.bind("find_stream", (uri) => {
		if (uri_data.hasOwnProperty(uri)) {
			return [Peer.uuid];
		} else {
			return null; // or []??
		}
	});

	p.proxy("source_calibration", (cb, uri) => {
		let peer = uri_data[uri].peer;
		if (peer) {
			peer.rpc("source_calibration", cb, uri);
		}
	});

	p.bind("set_pose", (uri, vec) => {
		//console.log("SET POSE");
		let peer = uri_data[uri].peer;
		if (peer) {
			uri_data[uri].pose = vec;
			peer.send("set_pose", uri, vec);
		}
	});

	p.bind("get_stream", (uri, N, rate, pid, dest) => {
		let peer = uri_data[uri].peer;
		if (peer) {
			peer.bind(uri, (rgb, depth) => {
				uri_data[uri].rgb = rgb;
				uri_data[uri].depth = depth;
				p.send(uri, rgb, depth);
			});
			peer.send("get_stream", uri, N, rate, [Peer.uuid], dest);
		}
	});
});

console.log("Listening or port 8080");
app.listen(8080);

