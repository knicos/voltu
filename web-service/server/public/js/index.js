
const checkIfLoggedIn = async () => {
    const token = window.localStorage.getItem('token')
    if(!token){
        console.log("You need to login")
    //User has a token saved in the browser
    }else{
        //validate that token
        const response = await fetch('http://localhost:8080/auth/validation/', {
            method: 'POST',
            headers: {'Authorization': token}
        })
        console.log(response)
        //Token is valid
        if(response.status === 200){
            console.log("SUCCESS")
            /*
            Most likely it will render a new HTML file
            */
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




//Creates thumbnail (image) for all available streams and adds them to div class='container'
const renderThumbnails = async () => {
    const thumbnails = await getAvailableStreams();
    console.log('THUMBNAILS', thumbnails)
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = '';
    console.log(containerDiv)
    for(var i=0; i<thumbnails.length; i++){
        const encodedURI = encodeURIComponent(thumbnails[i])
        try{
            const someData = await fetch(`http://localhost:8080/stream/rgb?uri=${encodedURI}`)
            console.log('SOME DATA', someData)
            if(!someData.ok){
                throw new Error('Image not found')
            }
            const myBlob = await someData.blob();
            console.log('BLOB', myBlob)
            const objectURL = URL.createObjectURL(myBlob);
            containerDiv.innerHTML += `<img src='${objectURL}' alt="Hups" width='500px'></img>`
        }catch(err){
            console.log("Couldn't create thumbnail");
            return 
        }
    }
}