const checkIfLoggedIn = async () => {
    const token = window.localStorage.getItem('token')
    console.log(token)
    if(!token){
        console.log("You need to login")
        renderLogin()
    }else{

        //Check if the token is valid
        const response = await fetch('http://localhost:8080/auth/validation', {
            method: 'POST',
            headers: {'Authorization': token}
        })
        console.log('RESPONSE', response)
        
        //Token is valid, show available streams
        if(response.status === 200){
            console.log("SUCCESS")
           renderThumbnails()
        }
    }
}

//Redirects the user to google authentication
const handleLogin = () => {
    window.location.href="/google";
}

/**
 * Returns a list of available streams
 */
const getAvailableStreams = async () => {
    const streamsInJson = await fetch('http://localhost:8080/streams');
    const streams = await streamsInJson.json();
    console.log('AVAILABLE', streams)
    return streams;
}

const videoPlayer = () => {
    const containerDiv = document.getElementById('container');
    const asd = 'yeahboiii'
    window.open(`http://localhost:8080/stream?uri=${asd}`)   
}



/**
 * Creates thumbnail (image) for all available streams and adds them to div class='container'
 */
const renderThumbnails = async () => {
    // const thumbnails = await getAvailableStreams();
    //console.log('THUMBNAILS', thumbnails)
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = '';
    console.log(containerDiv)
    for(var i=0; i<2; i++){
        // const encodedURI = encodeURIComponent(thumbnails[i])
        // console.log("THUMBNAIL[i]", thumbnails[i])
        // try{
        //     const someData = await fetch(`http://localhost:8080/stream/rgb?uri=${encodedURI}`)
        //     console.log('SOME DATA', someData)
        //     if(!someData.ok){
        //         throw new Error('Image not found')
        //     }
        //     const myBlob = await someData.blob();
        //     console.log('BLOB', myBlob)
        //     const objectURL = URL.createObjectURL(myBlob);
            containerDiv.innerHTML += createCard()
            // containerDiv.innerHTML += createCard(objectURL, i+4, encodedURI)
        // }catch(err){
        //     console.log("Couldn't create thumbnail");
        //     console.log(err) 
        // }
    }
}

/**
 * Renders button that will redirect to google login
 */
const renderLogin = () => {
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

// const createCard = (url, viewers, uri) => {
//     return `<div class='ftlab-card-component' >
//                 <img src='${url}' class="thumbnail-img" alt="Hups"></img>
//                 <p>Viewers: ${viewers}</p>
//                 <button onclick="window.location.href='/stream/${uri}'">button</button>
//             </div>`
// }

const createCard = () => {
    return `<div class='ftlab-card-component'>
                <img src='https://via.placeholder.com/250x150' class="thumbnail-img" width="250px" alt="Hups"></img>
                <p>Viewers: yes</p>
                <button onclick="window.location.href='/stream/URI'">button</button>
            </div>`
}

const cardLogic = () => {
    const cards = document.getElementsByClassName('ftlab-card-component');
    console.log("CARDS", cards)
}