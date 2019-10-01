import React, {Component} from 'react';

class Thumbnail extends Component {
    constructor(props){
        super(props)
        this.state = {imgSrc: ''}
    }

    componentDidMount(){
        this.renderThumbnails()
    }

    renderThumbnails = async () => {
        console.log('PROPS', this.props)
        const thumbs = this.props.thumbnail
        console.log(thumbs);
        const encodedURI = encodeURIComponent(thumbs)
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
            this.setState({imgSrc: 'Error while loading the picture'})
        }
    }
    render(){
        const val = this.state.imgSrc
        return (
            <div>
                {console.log('SRC', this.state.imgSrc)}
                <img src={val} width='500px'></img>
                <button onClick={() => {this.renderThumbnails()}}>Refresh</button>
            </div>
        )    
    }
}
 

export default Thumbnail;