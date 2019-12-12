const Peer = require('../../server/src/peer')
const VideoConverter = require('./lib/dist/video-converter');

let current_data = {};
let peer;
let player;

/**
 * Validates that the user is logged in by sending the token 
 */
checkIfLoggedIn = async () => {
    //     const token = window.localStorage.getItem('token')
    //     console.log(token)
    //     if(!token){
    //         console.log("You need to login")
    //         renderLogin()
    //     }else{

    //         //Check if the token is valid
    //         const response = await fetch('http://localhost:8080/auth/validation', {
    //             method: 'POST',
    //             headers: {'Authorization': token}
    //         })
    //         console.log('RESPONSE', response)
            
    //         //Token is valid, show available streams
    //         if(response.status === 200){
    //             console.log("SUCCESS")
                 renderThumbnails()

    //         }
    //     }
}

/**
 * Returns a list of available streams
 */
getAvailableStreams = async () => {
    try{
        const streamsInJson = await fetch('http://localhost:8080/streams');
        console.log(streamsInJson)
        const streams = await streamsInJson.json();
        console.log('AVAILABLE', streams)
        return streams;
    }catch(err){
        console.log(err)
    }
}


createVideoPlayer = () => {
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = `<h1>Stream ${current_data.uri} is live right here!</h1><br>
    <button onclick="renderThumbnails(); closeStream()">Go back</button>
    <button onclick="connectToStream()">Start Stream</button><br>
    <button onclick="webSocketTest()">WebSocket Test</button><br>
    <video id="ftlab-stream-video" width="640" height="360"></video>`;
    containerDiv.innerHTML += '<br>'
    containerDiv.innerHTML += ''
    createPeer();
    connectToStream();
}

/**
 * Creates thumbnail (image) for all available streams and adds them to div class='container'
 */
renderThumbnails = async () => {
    const thumbnails = await getAvailableStreams();
    // console.log('THUMBNAILS', thumbnails)
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = '';
    containerDiv.innerHTML = `<button onClick="configs()">change configs</button>`
    containerDiv.innerHTML += `<div class="ftlab-stream-thumbnails"></div>`
    // console.log(containerDiv)
    for(var i=0; i<thumbnails.length; i++){
        const encodedURI = encodeURIComponent(thumbnails[i])
        current_data.uri = thumbnails[i]
        console.log("THUMBNAIL[i]", thumbnails[i])
        try{
            const someData = await fetch(`http://localhost:8080/stream/rgb?uri=${encodedURI}`)
            console.log('SOME DATA', someData)
            if(!someData.ok){
                throw new Error('Image not found')
            }
            const myBlob = await someData.blob();
            console.log('BLOB', myBlob)
            const objectURL = URL.createObjectURL(myBlob);
            // containerDiv.innerHTML += createCard()
            containerDiv.innerHTML += createCard(objectURL, i+4)
        }catch(err){
            console.log("Couldn't create thumbnail");
            console.log(err) 
        }
    }
}


/** 
 * Method to create a single thumbnail
 */
createCard = (url, viewers) => {
    return `<div class='ftlab-card-component' >
                <img src='${url}' class="thumbnail-img" alt="Hups" width="250px"></img>
                <p>Viewers: ${viewers}</p>
                <button onclick="createVideoPlayer()">button</button>
            </div>`
}


createPeer = () => {
    const ws = new WebSocket('ws://localhost:8080/');
    ws.binaryType = "arraybuffer";
    peer = new Peer(ws)
}

webSocketTest = () => {
    peer.send("update_cfg", "ftl://utu.fi#reconstruction_default/0/renderer/cool_effect", "true")    
}


connectToStream = () => {
    const element = document.getElementById('ftlab-stream-video');
    const converter = new VideoConverter.default(element, 20, 6);

    peer.bind(current_data.uri, (latency, streampckg, pckg) => {
        if(pckg[0] === 2){
            function decode(value){
                converter.appendRawData(value);
            }
            decode(pckg[5]);
            converter.play();
        };
    })

    // Start the transaction
    peer.send("get_stream", (current_data.uri, 30, 0, current_data.uri));
}

closeStream = () => {
    peer.sock.close()
}



/**
 * **************
 * CONFIGURATIONS
 * **************
 */

current_data.configURI = "ftl://utu.fi#reconstruction_snap8/net"

configs = () => {
    const container = document.getElementById("container");
    container.innerHTML = `<div class="ftlab-configurations"></div>`;
    renderConfigOptions();
}


renderConfigOptions = () => {
    const input = `<p>input1</p><br>ftl://utu.fi#<input type="text">`
    const doc = document.getElementsByClassName('ftlab-configurations')[0];
    doc.innerHTML = input;
}

/**
 * 
 */
loadConfigs = async (str) => {
    const configURI = encodeURIComponent(`ftl://utu.fi#reconstruction_snap8${str}`);
    const uri = encodeURIComponent(current_data.uri)
    const rawResp = await fetch(`http://localhost:8080/stream/config?settings=${configURI}&uri=${uri}`)
    const response = await rawResp.json();
    const content = JSON.parse(response);
    container.innerHTML += `<p>${response}</p>`;
}

// current_data.configData = '{"peers": 1}';

/**
 * Method to send configurations to backend 
 */
saveConfigs = async () => {
    let {uri, configURI, configData} = current_data
    const rawResp = await fetch('http://localhost:8080/stream/config', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({peerURI: uri, configURI, data: configData, saveToCPP: true})
    });
    const content = await rawResp.json();
}