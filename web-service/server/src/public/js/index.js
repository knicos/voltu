const Peer = require('../../peer')

let current_data = {};
let peer;

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

videoPlayer = () => {
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = `<h1>Stream ${current_data.uri} is live right here!</h1><br><button onclick="renderThumbnails(); closeStream()">Go back</button><br>
    <canvas id="ftlab-stream-video" width="640" height="360"></canvas>`;
    containerDiv.innerHTML += '<br>'
    containerDiv.innerHTML += ''
    createPeer();
    setTimeout(connectToStream, 500)
    
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


// //FOR LAPTOP
// const renderThumbnails = async () => {
//     const containerDiv = document.getElementById('container')
//     containerDiv.innerHTML = '';
//     for(var i=0; i<2; i++){
//             containerDiv.innerHTML += createCard()
//     }
// }

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

//FOR DESKTOP
createCard = (url, viewers) => {
    return `<div class='ftlab-card-component' >
                <img src='${url}' class="thumbnail-img" alt="Hups" width="250px"></img>
                <p>Viewers: ${viewers}</p>
                <button onclick="videoPlayer()">button</button>
            </div>`
}


//FOR LAPTOP
// const createCard = () => {
//     return `<div class='ftlab-card-component'>
//                 <img src='https://via.placeholder.com/250x150' class="thumbnail-img" width="250px" alt="Hups"></img>
//                 <p>Viewers: yes</p>
//                 <button onclick="window.location.href='/stream?uri'">button</button>
//             </div>`
// }


createPeer = () => {
    const ws = new WebSocket('ws://localhost:8080/');
    ws.binaryType = "arraybuffer";
    peer = new Peer(ws)
    console.log("peer", peer)
}

/**
 *setTimeout 1s, ask for the amount of frames user has selected
 *
 *@param uri the uri where that should be called
 * 
 * */
connectToStream = () => {
    console.log(current_data.uri)
    const deocdedURI = decodeURIComponent(current_data.uri);
    peer.bind(deocdedURI, (latency, streampckg, pckg) => {
        console.log(pckg[0])
        if(pckg[0] === 0){
            const newBlob = new Blob( [pckg[5]], {type: "image/jpeg"});
            const canvas = document.getElementById("ftlab-stream-video");
            let modified = canvas.getContext("2d");
            let image = new Image();
            image.onload = () => {
                modified.drawImage(image, 0, 0)
            }
            image.src = URL.createObjectURL(newBlob)
        }
    })
    peer.send("get_stream", (current_data.uri, 10, 0, current_data.uri))
}

closeStream = () => {
    peer.sock.close()
}

const cardLogic = () => {
    const cards = document.getElementsByClassName('ftlab-card-component');
}

configs = () => {
    const container = document.getElementById("container");
    container.innerHTML = `<div class="ftlab-configurations"></div>`;
    let configContainer = document.getElementsByClassName("ftlab-configurations")[0];
}

renderConfigOptions = () => {
    const input = `<p>input1</p><br><input type="text">`
    const doc = document.getElementsByClassName('ftlab-configurations')[0];
    doc.innerHTML = input
}

updateConfigs = async () => {
    const rawResp = await fetch('http://localhost:8080/stream/config', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({board_size: [0, 5], square_size: 1, frame_delay: 5, num_frames: 10, URI: "current_data.uri"})
    });
    const content = await rawResp.json();
    console.log(content)
}

//current_data.configURI
//configURI = 
/**
 * current_data.configURI is a dropdown menu
 */

current_data.configs = 'ftl://utu.fi/stream/configurations/calibrations/default/board_size'
loadConfigs = async () => {
    const configURI = encodeURIComponent(current_data.configs);
    const uri = encodeURIComponent(current_data.uri)
    const rawResp = await fetch(`http://localhost:8080/stream/config?settings=${configURI}&uri=${uri}`)
    const content = await rawResp.json();
    console.log(content)
}