const Socket = require('../src/socket.js');
const assert = require('assert');
const net = require('net');
const binary = require('bops');

describe("Constructing a socket", function() {
	let server;
	
	beforeEach(() => {
		server = net.createServer(socket => {
			console.log("Client connected");
		});
		server.listen(9000, 'localhost');
	});
	
	it("Connects to a valid tcp uri", function(done) {
		let sock = new Socket("tcp://localhost:9000");
		sock.on('open', () => {
			console.log("OPEN");
			assert.equal(sock.isConnected(),true);
			sock.close();
			done();
		});
	});
	
	afterEach(() => {
		server.close(() => { console.log("Closed"); });
		server.unref();
	});
});

describe("Receiving messages on a tcp socket", function() {
	let server;
	
	beforeEach(() => {
		server = net.createServer(socket => {
			console.log("Client connected");
			socket.write(Buffer.from([8,0,0,0,44,0,0,0,23,0,0,0]));
		});
		server.listen(9001, 'localhost');
	});
	
	it("receives valid short message", function(done) {
		let sock = new Socket("tcp://localhost:9001");
		sock.on(44, (size, data) => {
			// TODO Parse the data...
			assert.equal(binary.readInt32LE(data,0), 23);
			console.log("Received data....");
			sock.close();
			done();
		});
	});
	
	afterEach(() => {
		server.close(() => { console.log("Closed"); });
		server.unref();
	});
});

