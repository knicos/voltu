import React, { useState, useEffect } from 'react';
//import './App.css'
//import {Route, BrowserRouter, Link} from 'react-router-domm
import Streams from './Streams'
import Login from './Login'

const App = () => {

    const [user, setUser] = useState(null);
    const [streams, setStreams] = useState(null);


    // useEffect(() => {
    //     const loggedUserJSON = window.localStorage.getItem('token');
    //     if(loggedUserJSON){
    //         const user = JSON.parse(loggedUserJSON);
    //         setUser(user);
    //     }
    // })

    /**
     * TODO: create a list available of streams using useEffect.
     * Each index is an object (stream) that has a name and a picture.
     * When making the request for the objects the same request 
     * authenticates the users JWT
     * 
     */

    const handleLogin = () => {
        //BEFORE BUILD CHANGE THE URL TO /google
        window.location.href="/google";
    }

    const clearCookies = () => {
        window.localStorage.clear(); 
        window.location.reload();
    }

    // if(window.localStorage.getItem('token')){
    //     return (
    //        
    //     )
    // }
    return (
        <Streams clearCookies={clearCookies}/>
        // <Login handleLogin={handleLogin} />
    )
}

export default App;   