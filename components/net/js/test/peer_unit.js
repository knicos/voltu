const Peer = require('../src/peer.js');
const assert = require('chai').assert;
const net = require('net');
//const binary = require('bops');
const WebSocket = require('ws');

describe("Peer()", function() {
	let server;
	let wss;
	let dobadhandshake = false;
	
	beforeEach(() => {
		dobadhandshake = false;
		server = net.createServer(socket => {
			socket.on('error', ()=> {});
			if (dobadhandshake) {
				socket.write(Buffer.from([44,55,33,22,23,44,87]));
			} else {
				socket.write(Buffer.from([0x12,0x09,0x64,0x53,0x00,0x34,0x99,0x10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,67,67,67]));
			}
		});
		server.listen(9000, 'localhost');
		
		wss = new WebSocket.Server({ port: 9001 });
		wss.on('connection', (ws) => {
			ws.on('error', ()=> {});
			if (dobadhandshake) {
				ws.send(Buffer.from([44,55,33,22,23,44,87]));
			} else {
				ws.send(Buffer.from([0x12,0x09,0x64,0x53,0x00,0x34,0x99,0x10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,67,67,67]));
			}
		});
	});
	
	context("with a valid connection uri and handshake", () => {
		it("make a tcp connection", function(done) {
			let sock = new Peer("tcp://localhost:9000");
			sock.on('open', () => {
				assert.isOk(sock.isConnected());
				sock.close();
				done();
			});
		});
		
		it("make a websocket connection", function(done) {
			let sock = new Socket("ws://localhost:9001");
			sock.on('open', () => {
				assert.isOk(sock.isConnected());
				sock.close();
				done();
			});
		});
	});
	
	context("with a valid uri but bad handshake", (done) => {
		it("should reject the connection", () => {
			dobadhandshake = true;
			let sock = new Socket("tcp://localhost:9000");
			sock.on('error', (errno) => {
				assert.equal(errno, Socket.ERROR_BADHANDSHAKE);
				assert.isOk(sock.isValid());
				assert.isNotOk(sock.isConnected());
				done();
			});
		});
	});
	
	context("with an invalid connection uri", () => {
		it("should give protocol error", () => {
			let diderror = false;
			let sock = new Socket("xyz://localhost:9000");
			sock.on('error', (errno) => {
				diderror = true;
				assert.equal(errno, Socket.ERROR_BADPROTOCOL);
				assert.isNotOk(sock.isValid());
			});
			assert.isOk(diderror);
		});
		
		it("should give host error", (done) => {
			let sock = new Socket("tcp://blah.blah:9000");
			sock.on('error', (errno) => {
				assert.equal(errno, Socket.ERROR_BADHOST);
				assert.isNotOk(sock.isValid());
				done();
			});
		});
		
		it("should give a malformed uri error", () => {
			let diderror = false;
			let sock = new Socket("helloworld");
			sock.on('error', (errno) => {
				diderror = true;
				assert.equal(errno, Socket.ERROR_MALFORMEDURI);
				assert.isNotOk(sock.isValid());
			});
			assert.isOk(diderror);
		});
	});
	
	afterEach(() => {
		server.close();
		server.unref();
		
		wss.close();
	});
});

describe("Receiving messages on a tcp socket", function() {
	let server;
	
	beforeEach(() => {
		server = net.createServer(socket => {
			// Handshake first
			socket.write(Buffer.from([0x12,0x09,0x64,0x53,0x00,0x34,0x99,0x10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,67,67,67]));
			socket.write(Buffer.from([8,0,0,0,44,0,0,0,23,0,0,0]));
		});
		server.listen(9001, 'localhost');
	});
	
	it("receives valid short message", function(done) {
		let sock = new Socket("tcp://localhost:9001");
		sock.on(44, (size, data) => {
			assert.equal(binary.readInt32LE(data,0), 23);
			console.log("Received data....");
			sock.close();
			done();
		});
	});
	
	afterEach(() => {
		server.close();
		server.unref();
	});
});


