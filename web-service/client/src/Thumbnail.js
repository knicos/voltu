import React, {Component} from 'react';

const Thumbnail = ({thumbnail}) => {
    const renderThumbnails = async () => {
        let returnVal = "err";
        const encodedURI = encodeURIComponent(thumbnail)
        console.log(encodedURI)
        try{
            const someData = await fetch(`http://localhost:8080/stream/rgb?uri=${encodedURI}`)
            console.log(someData)
            if(!someData.ok){
                throw new Error('Vitun vitun vittu');
            }
            const myBlob = await someData.blob()
            console.log('MYBLOB', myBlob)
            const objectURL = URL.createObjectURL(myBlob);
            console.log('URL ', objectURL);
            return objectURL;
        } catch(err){
            console.log('Kurwavaara:', err);
        }
        console.log('RETURN VALUE', returnVal)
        return returnVal;
    }
    return (
        <div>
            <img src={renderThumbnails()} alt="Hups"></img>
        </div>
    )    
}
 

export default Thumbnail;