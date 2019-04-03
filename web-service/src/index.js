const express = require('express');
const app = express();
const expressWs = require('express-ws')(app);

app.get('/', (req, res) => {
	res.end();
});

app.ws('/', (ws, req) => {
	console.log("New web socket request");
	// SEND Handshake
	ws.on('message', (msg) => {
		console.log("Message", msg);
	});
	ws.on('error', () => {
		console.log("Error");
	});
	ws.on('close', () => {
		console.log("Close");
	});
});

console.log("Listening or port 3000");
app.listen(3000);

