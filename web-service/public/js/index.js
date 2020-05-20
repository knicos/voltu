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
        const streamsInJson = await fetch(`./streams`);
        const streams = await streamsInJson.json();
        console.log('AVAILABLE', streams)
        return streams;
    }catch(err){
        console.log(err)
    }
}


createVideoPlayer = () => {
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = `<h1>Stream from source ${current_data.uri}</h1><br>
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
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = '';
    containerDiv.innerHTML = `<button onClick="configs()">change configs</button>`
    containerDiv.innerHTML += `<div class="ftlab-stream-thumbnails"></div>`
    if(thumbnails.length === 0){
        containerDiv.innerHTML = `<h3>No streams running currently</h3>`
    }else{
        for(var i=0; i<thumbnails.length; i++){
            const encodedURI = encodeURIComponent(thumbnails[i])
            current_data.uri = thumbnails[i]
            try{
                const someData = await fetch(`./stream/rgb?uri=${encodedURI}`)
                if(!someData.ok){
                    throw new Error('Image not found')
                }
                const myBlob = await someData.blob();
                const objectURL = URL.createObjectURL(myBlob);
                // containerDiv.innerHTML += createCard()
                containerDiv.innerHTML += createCard(objectURL, i+4)
            }catch(err){
                console.log("Couldn't create thumbnail");
                console.log(err) 
            }
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
    // FOR PRODUCTION
    const ws = new WebSocket("ws://" + location.host + ":" + (location.port == "" ? "80" : location.port) + location.pathname);
    //const ws = new WebSocket("ws://localhost:8080")
    ws.binaryType = "arraybuffer";
    peer = new Peer(ws)
}

webSocketTest = () => {
    peer.send("update_cfg", "ftl://utu.fi#reconstruction_default/0/renderer/cool_effect", "true")    
}


connectToStream = () => {
    const element = document.getElementById('ftlab-stream-video');
    let converter = null;

    let rxcount = 0;
    let ts = 0;
    let dts = 0;

    peer.bind(current_data.uri, (latency, streampckg, pckg) => {
        if(pckg[0] === 2){
            rxcount++;
            if (rxcount >= 25) {
                rxcount = 0;
                peer.send(current_data.uri, 0, [1,0,255,0],[255,7,35,0,0,Buffer.alloc(0)]);
                //peer.send(current_data.uri, 0, [255,7,35,0,0,Buffer.alloc(0)], [1,0,255,0]);
            }

            if (converter) {
                function decode(value){
                    converter.appendRawData(value);
                }
                decode(pckg[5]);
                converter.play();
            } else {
                if (ts > 0) {
                    dts = streampckg[0] - ts;
                    console.log("Framerate = ", 1000/dts);
                    converter = new VideoConverter.default(element, 30, 1);
                }
                ts = streampckg[0];
            }
        };
    })

    // Start the transaction
    //peer.send("get_stream", (current_data.uri, 30, 0, current_data.uri));

    peer.rpc("find_stream", (res) => {
        peer.send(current_data.uri, 0, [1,0,255,0],[255,7,35,0,0,Buffer.alloc(0)]);
        //peer.send(current_data.uri, [255,7,35,0,0,Buffer.alloc(0)], [1,0,255,0]);
    }, current_data.uri);
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
    const rawResp = await fetch(`./stream/config?settings=${configURI}&uri=${uri}`)
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
    const rawResp = await fetch('./stream/config', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({peerURI: uri, configURI, data: configData, saveToCPP: true})
    });
    const content = await rawResp.json();
}