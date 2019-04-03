const Socket = require('../src/socket.js');
const assert = require('assert');
const net = require('net');

describe("Constructing a socket", function() {
	
	let server = net.createServer(socket => {
		console.log("Client connected");
	});
	server.listen(9000, 'localhost');
	
	it("Connects to a valid tcp uri", function(done) {
		let sock = new Socket("tcp://localhost:9000");
		sock.on('connect', () => {
			assert.equal(sock.isConnected(),true);
			sock.close();
			done();
		});
	});
	
	server.close(() => { console.log("Closed"); });
});

describe("Receiving messages on a socket", function() {
	
	let server = net.createServer(socket => {
		console.log("Client connected");
		server.write(Buffer.from('helloworld'));
	});
	server.listen(9000, 'localhost');
	
	it("Connects to a valid tcp uri", function(done) {
		let sock = new Socket("tcp://localhost:9000");
		sock.on(44, (data) => {
			// TODO Parse the data...
			console.log("Received data....");
			done();
			sock.close();
		});
	});
	
	server.close(() => { console.log("Closed"); });
});

