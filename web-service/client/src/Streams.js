import React, { useState, useEffect } from 'react'
import Stream from './Stream'
import testData from './seeds'

/* This file will contain list of streams.
The user is able to select which stream he/she will WebAuthentication.
The user will be redirected to Stream.js file  
*/



const Streams = ({clearCookies}) => {
    const [thumbnails, setThumbnails] = useState([]);
    
    useEffect( async () => {
        const jsonThumbnails = await fetch('/streams/');
        const realThumbnails = await jsonThumbnails.json();
        setThumbnails(realThumbnails);
        console.log('THUMBNAILS', thumbnails)
    }, [])
    /**
     * Fetch the thumbnails
     * setInterval() fetch every 1 second
     */

    const fetchThumbnails = async () => {
        const jsonThumbnails = await fetch('/streams/');
        const realThumbnails = await jsonThumbnails.json();
        return realThumbnails;
    }

    const renderThumbnails = async () => {
        //updates all available thumbnail URIs
        const thumbs = await fetchThumbnails()
        setThumbnails((thumbs));
        console.log(thumbnails[0]);
        //Problem possibly here, it doesn't encode it correctly?
        const encodedURL = encodeURI(thumbnails[0]);
        console.log('ENCODED URL', encodedURL);
        try{
            const someData = await fetch(`/stream/rgb?uri=${encodedURL}`)
            if(!someData){
                throw new Error('Vitun vitun vittu');
            }
            const myBlob = await someData.blob
            console.log('MYBLOB', myBlob)
            const objectURL = URL.createObjectURL(myBlob);
            console.log('URL ', objectURL);
        } catch(err){
            console.log('Kurwavaara:', err);
        }
        
        return thumbnails;
    }



    return(
        <div style={{'margin': 'auto', 'textAlign': 'center'}}>
            <h1>Streams component works!!</h1>
            <h2>Namibia here we come!</h2>
            <button onClick={clearCookies}>Logout</button>
            <br/>
            <button onClick={renderThumbnails}></button>
        </div>
    )
}

export default Streams;
/*
  Server has web socket method "list_streams" that returns the keys for all streams.

  TODO:
    React Router
        Path might be something like /streams
    How will the component get a stream thumbnail?
        After getting the stream keys it renders bunch of child components 
        that each get one key. After getting the key the individual child component makes 
        a request for "get_stream" method.
    
    How will the component connect to a stream?
        When one of the thumbnails is clicked, it redirects to path /stream and gives it the specific streams object key as props


 */