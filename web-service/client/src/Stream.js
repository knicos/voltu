import React from 'react'
/*
Component for individual stream
    Router will render to this component in path /stream
    Gets specific streams object key as prop
*/

const Stream = () => {
    return(
        <div>
            <video controls autoPlay src='https://s3.amazonaws.com/codecademy-content/courses/React/react_video-eek.mp4'/>
        </div>
    )
}

export default Stream;