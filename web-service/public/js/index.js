const Peer = require('../../server/src/peer')
const VideoConverter = require('./lib/dist/video-converter');

let current_data = {};
let peer;
let player;
console.log(VideoConverter);

/**
 * Validates that the user is logged in
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

//Redirects the user to google authentication
handleLogin = () => {
    window.location.href="/google";
}

/**
 * Returns a list of available streams
 */
getAvailableStreams = async () => {
    try{
        const streamsInJson = await fetch('http://localhost:8080/streams');
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
    console.log("PLAYER", player)
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
 * Renders button that will redirect to google login
 */
renderLogin = () => {
    const containerDiv = document.getElementById('container');
        containerDiv.innerHTML = 
        `<div id='Login'>
            <h2>Welcome to Future Technology Lab</h2>
            <h3>Please login!</h3>
            <a className="button" onClick="handleLogin()">
                <div>
                    <span class="svgIcon t-popup-svg">
                        <svg class="svgIcon-use" width="25" height="37" viewBox="0 0 25 25">
                            <g fill="none" fill-rule="evenodd">
                            <path d="M20.66 12.693c0-.603-.054-1.182-.155-1.738H12.5v3.287h4.575a3.91 3.91 0 0 1-1.697 2.566v2.133h2.747c1.608-1.48 2.535-3.65 2.535-6.24z" fill="#4285F4"/>
                            <path d="M12.5 21c2.295 0 4.22-.76 5.625-2.06l-2.747-2.132c-.76.51-1.734.81-2.878.81-2.214 0-4.088-1.494-4.756-3.503h-2.84v2.202A8.498 8.498 0 0 0 12.5 21z" fill="#34A853"/>
                            <path d="M7.744 14.115c-.17-.51-.267-1.055-.267-1.615s.097-1.105.267-1.615V8.683h-2.84A8.488 8.488 0 0 0 4 12.5c0 1.372.328 2.67.904 3.817l2.84-2.202z" fill="#FBBC05"/>
                            <path d="M12.5 7.38c1.248 0 2.368.43 3.25 1.272l2.437-2.438C16.715 4.842 14.79 4 12.5 4a8.497 8.497 0 0 0-7.596 4.683l2.84 2.202c.668-2.01 2.542-3.504 4.756-3.504z" fill="#EA4335"/>
                            </g>
                        </svg>
                    </span>
                    <span class="button-label">Sign in with Google</span>
                </div>
            </a>
        </div>`
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
    console.log(current_data.uri)
    peer.send("update_cfg", "ftl://utu.fi#reconstruction_default/0/renderer/cool_effect", "true")    
}


connectToStream = () => {
    const element = document.getElementById('ftlab-stream-video');
    console.log(VideoConverter)
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
    console.log(content)
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
    console.log(content)
}