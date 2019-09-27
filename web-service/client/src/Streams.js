import React, { useState, useEffect } from 'react'
import Stream from './Stream'
import testData from './seeds'

/* This file will contain list of streams.
The user is able to select which stream he/she will WebAuthentication.
The user will be redirected to Stream.js file  
*/



const Streams = ({clearCookies}) => {
    const [thumbnails, setThumbnails] = useState([]);

    useEffect(() => {
        setThumbnails(testData.thumbnail);
        console.log('streams')
    }, [])
    
    const renderThumbnails = () => {
        const photos = thumbnails.map(i => <div style={{'padding-top': '25px'}}><img alt='robots' src={i.video} width='150px'/><p style={{'margin': 'none'}}>viewers: {i.viewers}</p></div>)
        return photos;
    }

    return(
        <div style={{'margin': 'auto', 'textAlign': 'center'}}>
            <h1>Streams component works!!</h1>
            <h2>Namibia here we come!</h2>
            <button onClick={clearCookies}>Logout</button>
            <br />
            {renderThumbnails()}
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