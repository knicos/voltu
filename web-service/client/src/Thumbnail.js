import React, {Component} from 'react';

class Thumbnail extends Component {
    constructor(props){
        super(props)
        this.state = {imgSrc: ''}
    }

    componentDidMount(){
        this.renderThumbnails()
    }

    fetchThumbnails = async () => {
        const jsonThumbnails = await fetch('http://localhost:8080/streams/')
        const realThumbnails = await jsonThumbnails.json();
        console.log('REAL THUMBNAILS', realThumbnails)
        return realThumbnails;
    }

    renderThumbnails = async () => {
        let returnVal = "err";
        console.log(this.props)
        const thumbs = await this.fetchThumbnails()
        console.log(thumbs);
        const encodedURI = encodeURIComponent(thumbs[0])
        console.log('ENCODED', encodedURI)
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
            this.setState({imgSrc: objectURL});
        } catch(err){
            console.log('Kurwavaara:', err);
        }
    }
    render(){
        const val = this.state.imgSrc
        return (
            <div>
                {console.log('SRC', this.state.imgSrc)}
                <img src={val} width='500px'></img>
                <button onClick={() => this.renderThumbnails()}></button>
            </div>
        )    
    }
}
 

export default Thumbnail;