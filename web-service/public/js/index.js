const Peer = require('../../server/src/peer')
const msgpack = require('msgpack5')();
const rematrix = require('rematrix');
const THREE = require('three');
const MUXJS = require('mux.js');
const MP4 = MUXJS.mp4.generator;
const H264Stream = MUXJS.codecs.h264.H264Stream;
//const VIDEO_PROPERTIES = require('../../node_modules/mux.js/lib/constants/video-properties.js');

const VIDEO_PROPERTIES = [
	'width',
	'height',
	'profileIdc',
	'levelIdc',
	'profileCompatibility',
	'sarRatio'
  ];
  

let current_data = {};
let peer;

/**
 * Validates that the user is logged in by sending the token 
 */
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

/**
 * Returns a list of available streams
 */
getAvailableStreams = async () => {
    try{
        const streamsInJson = await fetch(`./streams`);
        const streams = await streamsInJson.json();
        console.log('AVAILABLE', streams)
        return streams;
    }catch(err){
        console.log(err)
    }
}


createVideoPlayer = () => {
	const containerDiv = document.getElementById('container');
	containerDiv.innerHTML = '';
    /*containerDiv.innerHTML = `<h1>Stream from source ${current_data.uri}</h1><br>
        <button onclick="renderThumbnails(); closeStream()">Go back</button>
        <button onclick="connectToStream('${current_data.uri}')">Start Stream</button><br>
        <button onclick="webSocketTest()">WebSocket Test</button><br>
        <video id="ftlab-stream-video" width="640" height="360"></video>`;
    containerDiv.innerHTML += '<br>'
    containerDiv.innerHTML += ''*/
    createPeer();
	//connectToStream();
	window.ftlstream = new FTLStream(peer, current_data.uri, containerDiv);
}

/**
 * Creates thumbnail (image) for all available streams and adds them to div class='container'
 */
renderThumbnails = async () => {
    const thumbnails = await getAvailableStreams();
    const containerDiv = document.getElementById('container')
    containerDiv.innerHTML = '';
    containerDiv.innerHTML = `<button onClick="configs()">change configs</button>`
    containerDiv.innerHTML += `<div class="ftlab-stream-thumbnails"></div>`
    if(thumbnails.length === 0){
        containerDiv.innerHTML = `<h3>No streams running currently</h3>`
    }else{
        for(var i=0; i<thumbnails.length; i++){
            const encodedURI = encodeURIComponent(thumbnails[i])
            current_data.uri = thumbnails[i]
            try{
                const someData = await fetch(`./stream/rgb?uri=${encodedURI}`)
                if(!someData.ok){
                    throw new Error('Image not found')
                }
                const myBlob = await someData.blob();
                const objectURL = URL.createObjectURL(myBlob);
                // containerDiv.innerHTML += createCard()
                containerDiv.innerHTML += createCard(objectURL, i+4)
            }catch(err){
                console.log("Couldn't create thumbnail");
                console.log(err) 
            }
        }
    }
}


/** 
 * Method to create a single thumbnail
 */
createCard = (url, viewers) => {
    return `<div class='ftlab-card-component' >
                <img src='${url}' class="thumbnail-img" alt="Hups" width="250px"></img>
                <p>Viewers: ${viewers}</p>
                <button onclick="createVideoPlayer()">button</button>
            </div>`
}


createPeer = () => {
	// FOR PRODUCTION
	console.log("HOST", location.host);
    const ws = new WebSocket("ws://" + location.host + location.pathname);
	//const ws = new WebSocket("ws://localhost:8080")
    ws.binaryType = "arraybuffer";
    peer = new Peer(ws)
}

webSocketTest = () => {
    peer.send("update_cfg", "ftl://utu.fi#reconstruction_default/0/renderer/cool_effect", "true")    
}

function FTLFrameset(id) {
	this.id = id;
	this.sources = {};
}

function getNALType(data) {
	return (data.length > 4) ? data.readUInt8(4) & 0x1F : 0;
}

function isKeyFrame(data) {
	return getNALType(data) == 7;  // SPS
}

function concatNals(sample) {
	let length = sample.size;
	let data = new Uint8Array(length);
	let view = new DataView(data.buffer);
	let dataOffset = 0;

	for (var i=0; i<sample.units.length; ++i) {
		view.setUint32(dataOffset, sample.units[i].data.byteLength);
        dataOffset += 4;
        data.set(sample.units[i].data, dataOffset);
        dataOffset += sample.units[i].data.byteLength;
	}

	sample.data = data;
}

var createDefaultSample = function() {
	return {
	  units: [],
	  data: null,
	  size: 0,
	  compositionTimeOffset: 1,
	  duration: 0,
	  dataOffset: 0,
	  flags: {
		isLeading: 0,
		dependsOn: 1,
		isDependedOn: 0,
		hasRedundancy: 0,
		degradationPriority: 0,
		isNonSyncSample: 1
	  },
	  keyFrame: true
	};
  };

function FTLStream(peer, uri, element) {
	this.uri = uri;
	this.peer = peer;

	this.current = "";
	this.current_fs = 0;
	this.current_source = 0;
	this.current_channel = 0;

	this.framesets = {};

	this.handlers = {};

	//this.elements_ = {};
	//this.converters_ = {};

	//const element = document.getElementById('ftlab-stream-video');
	this.outer = element;
	this.outer.classList.add("ftl");
	this.outer.classList.add("container");
	this.element = document.createElement("VIDEO");
	this.element.setAttribute("width", 640);
	this.element.setAttribute("height", 360);
	this.element.setAttribute("controls", true);
	this.element.style.display = "none";
	this.element.classList.add("ftl");
	this.element.id = "ftl-video-element";
	this.outer.appendChild(this.element);

	//this.player = videojs('ftl-video-element');
	//this.player.vr({projection: '360'});

	if (false) {
		this.camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 1, 1100 );
	} else {
		this.camera = new THREE.OrthographicCamera(window.innerWidth/-2, window.innerWidth/2, window.innerHeight/2, window.innerHeight/-2, 1, 4);
	}
	this.camera.target = new THREE.Vector3( 0, 0, 0 );

	this.scene = new THREE.Scene();

	var geometry;
	
	if (false) {
		geometry = new THREE.SphereBufferGeometry( 500, 60, 40 );
	} else {
		geometry = new THREE.PlaneGeometry(1280, 720, 32);
	}
	// invert the geometry on the x-axis so that all of the faces point inward
	geometry.scale( - 1, 1, 1 );

	var texture = new THREE.VideoTexture( this.element );
	var material = new THREE.MeshBasicMaterial( { map: texture } );

	this.mesh = new THREE.Mesh( geometry, material );

	this.scene.add( this.mesh );

	this.renderer = new THREE.WebGLRenderer();
	this.renderer.setPixelRatio( window.devicePixelRatio );
	this.renderer.setSize( window.innerWidth, window.innerHeight );
	this.outer.appendChild( this.renderer.domElement );

	var me = this;

	this.isUserInteracting = false;
	this.onPointerDownPointerX = 0;
	this.onPointerDownPointerY = 0;
	this.onPointerDownLon = 0;
	this.onPointerDownLat = 0;
	this.lon = 0;
	this.lat = 0;
	this.distance = 2.0;

	this.overlay = document.createElement("DIV");
	this.overlay.classList.add("ftl");
	this.overlay.classList.add("overlay");
	this.overlay.setAttribute("tabindex","0");
	this.outer.appendChild(this.overlay);

	this.overlay.addEventListener('mousedown', (event) => {
		event.preventDefault();

		this.isUserInteracting = true;

		this.onPointerDownPointerX = event.clientX;
		this.onPointerDownPointerY = event.clientY;

		this.onPointerDownLon = this.lon;
		this.onPointerDownLat = this.lat;
	});

	this.overlay.addEventListener('mousemove', (event) => {
		if ( this.isUserInteracting === true ) {
			//this.lon = ( this.onPointerDownPointerX - event.clientX ) * 0.1 + this.onPointerDownLon;
			//this.lat = ( this.onPointerDownPointerY - event.clientY ) * 0.1 + this.onPointerDownLat;

			this.rotationX += event.movementY * (1/25) * 5.0;
			this.rotationY -= event.movementX * (1/25) * 5.0;
			this.updatePose();
		}
	});

	this.overlay.addEventListener('mouseup', (event) => {
		this.isUserInteracting = false;
	});

	this.overlay.addEventListener('wheel', (event) => {
		event.preventDefault();
		this.distance += event.deltaY * 0.05;
		this.distance = THREE.MathUtils.clamp( this.distance, 1, 50 );
	});

	function update() {
		me.lat = Math.max( - 85, Math.min( 85, me.lat ) );
		let phi = THREE.MathUtils.degToRad( 90 - me.lat );
		let theta = THREE.MathUtils.degToRad( me.lon );

		//me.camera.position.x = me.distance * Math.sin( phi ) * Math.cos( theta );
		//me.camera.position.y = me.distance * Math.cos( phi );
		//me.camera.position.z = me.distance * Math.sin( phi ) * Math.sin( theta );

		me.camera.position.x = 0;
		me.camera.position.y = 0;
		me.camera.position.z = -2;

		me.camera.lookAt( me.camera.target );

		me.renderer.render( me.scene, me.camera );

	}

	function animate() {

		requestAnimationFrame( animate );
		update();

	}

	animate();

	this.play_button = document.createElement("BUTTON");
	this.play_button.innerHTML = "Play";
	this.play_button.classList.add("ftl");
	this.play_button.classList.add("play");
	this.play_button.onclick = () => {
		this.start(0,0,0);
	}
	this.overlay.appendChild(this.play_button);

	this.pause_button = document.createElement("BUTTON");
	this.pause_button.innerHTML = "Pause";
	this.pause_button.classList.add("ftl");
	this.pause_button.classList.add("pause");
	this.pause_button.onclick = () => {
		this.pause();
	}
	this.overlay.appendChild(this.pause_button);

	this.paused = false;
	this.active = false;

	this.overlay.addEventListener('keydown', (event) => {
		console.log(event);
		switch(event.code) {
		case "KeyW"		: this.translateZ += 0.05; this.updatePose(); break;
		case "KeyS"		: this.translateZ -= 0.05; this.updatePose(); break;
		case "KeyA"		: this.translateX -= 0.05; this.updatePose(); break;
		case "KeyD"		: this.translateX += 0.05; this.updatePose(); break;
		}
	});

	/*this.element.onmousemove = (event) => {
		console.log(event);
		if (event.buttons == 1) {
			this.rotationX += event.movementY * (1/25) * 5.0;
			this.rotationY -= event.movementX * (1/25) * 5.0;
			this.updatePose();
		}
	}*/

	this.rotationX = 0;
	this.rotationY = 0;
	this.rotationZ = 0;
	this.translateX = 0;
	this.translateY = 0;
	this.translateZ = 0;

	//this.element.onclick = () => {
		//let pose = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
		//this.rotation += 10;
		//let pose = rematrix.rotateZ(this.rotation);
		//this.setPose(pose);
	//}

	//this.converter = null;
	
	/*this.converter = new JMuxer({
		node: 'ftl-video-element',
		mode: 'video',
		//fps: 1000/dts,
		fps: 30,
		flushingTime: 1,
		clearBuffer: false
	});*/

    let rxcount = 0;
    let ts = 0;
	let dts = 0;

	let seen_keyframe = false;
	let init_seg = false;

	this.h264 = new H264Stream();

	this.doAppend = function(data) {
		if (this.sourceBuffer.updating) {
			this.queue.push(data);
		} else {
			//console.log("Direct append: ", data);
			this.sourceBuffer.appendBuffer(data);
		}
	}

	this.h264.on('data', (nalUnit) => {
		//console.log("NAL", nalUnit);
		//this.segstream.push(data);

		//trackDecodeInfo.collectDtsInfo(track, nalUnit);

		// record the track config
		if (nalUnit.nalUnitType === 'seq_parameter_set_rbsp') {
			this.track.config = nalUnit.config;
			this.track.sps = [nalUnit.data];

			VIDEO_PROPERTIES.forEach(function(prop) {
				this.track[prop] = nalUnit.config[prop];
			}, this);
		}

		if (nalUnit.nalUnitType === 'pic_parameter_set_rbsp') {
			//pps = nalUnit.data;
			this.track.pps = [nalUnit.data];
		}

		if (!init_seg && this.track.sps && this.track.pps) {
			init_seg = true;
			console.log("Init", this.track);
			this.doAppend(MP4.initSegment([this.track]));
		}

		let keyFrame = nalUnit.nalUnitType == 'slice_layer_without_partitioning_rbsp_idr';

		// buffer video until flush() is called
		//nalUnits.push(nalUnit);
		//if (keyFrame || !nalUnit.hasOwnProperty("nalUnitType")) {
			/*this.track.samples.push({
				data: nalUnit.data,
				duration: 1,
				pts: nalUnit.pts,
				dts: nalUnit.dts,
				flags: {
					isLeading: 0,
					isDependedOn: 0,
					hasRedundancy: 0,
					degradPrio: 0,
					isNonSyncSample: keyFrame ? 0 : 1,
					dependsOn: keyFrame ? 2 : 1,
					paddingValue: 0,
				}
			});*/

			let sample = this.track.samples[0];
			sample.units.push(nalUnit);
			sample.size += nalUnit.data.byteLength + 4;

			sample.keyFrame &= keyFrame;
			
			if (keyFrame) {
				sample.flags.isNonSyncSample = 0;
				sample.flags.dependsOn = 2;
			}
		//}
	});

	this.track = {
		timelineStartInfo: {
			baseMediaDecodeTime: 0
		},
		baseMediaDecodeTime: 0,
		id: 0,
		codec: 'avc',
		type: 'video',
		samples: [],
		duration: 0
	};
	
	this.mediaSource = new MediaSource();
	//this.element.play();
	this.sourceBuffer = null;

	this.element.addEventListener('pause', (e) => {
		console.log("pause");
		this.active = false;
	});

	this.element.addEventListener('play', (e) => {
		console.log("Play");
		init_seg = false;
		seen_keyframe = false;
		ts = 0;
		this.track.baseMediaDecodeTime = 0;
		this.sequenceNo = 0;
		this.active = true;
		this.start(0,0,0);
	});

	this.mediaSource.addEventListener('sourceopen', (e) => {
		console.log("Source Open");
		URL.revokeObjectURL(this.element.src);
		console.log(this.mediaSource.readyState);
		this.sourceBuffer = e.target.addSourceBuffer(this.mime);
		this.sourceBuffer.mode = 'sequence';
		this.active = true;

		this.sourceBuffer.addEventListener('error', (e) => {
			console.error("SourceBuffer: ", e);
			this.active = false;
		});

		this.sourceBuffer.addEventListener('updateend', () => {
			if (this.queue.length > 0) {
				let s = this.queue[0];
				this.queue.shift();
				//console.log("Append", s);
				this.sourceBuffer.appendBuffer(s);
			}
		});
	});

	this.mime = 'video/mp4; codecs="avc1.640028"';
	this.queue = [];
	this.sequenceNo = 0;

	this.element.src = URL.createObjectURL(this.mediaSource);

    this.peer.bind(uri, (latency, streampckg, pckg) => {
		if (this.paused || !this.active) {
			return;
		}

        if(pckg[0] === 2){  // H264 packet.
			let id = "id-"+streampckg[1]+"-"+streampckg[2]+"-"+streampckg[3];

			if (this.current == id) {
				rxcount++;
				if (rxcount >= 25) {
					rxcount = 0;
					peer.send(uri, 0, [1,0,255,0],[255,7,35,0,0,Buffer.alloc(0)]);
					//peer.send(current_data.uri, 0, [255,7,35,0,0,Buffer.alloc(0)], [1,0,255,0]);
				}

				//console.log("NALU", getNALType(pckg[5]));

				if (!seen_keyframe) {
					if (isKeyFrame(pckg[5])) {
						console.log("Key frame ", streampckg[0]);
						seen_keyframe = true;
					}
				}
			
				if (seen_keyframe) {
					if (ts == 0) ts = streampckg[0];
					//if (this.track.samples.length > 0) console.error("Unfinished sample");
					dts += streampckg[0]-ts;

					this.track.samples.push(createDefaultSample());

					this.h264.push({
						type: 'video',
						dts: dts,
						pts: streampckg[0],
						data: pckg[5],
						trackId: 0
					});
					this.h264.flush();

					let sample = this.track.samples[0];
					/*if (sample.keyFrame) {
						sample.flags.isNonSyncSample = 0;
						sample.flags.dependsOn = 2;
					}*/

					concatNals(sample);
					let delta = (streampckg[0]-ts)*90;
					sample.duration = (delta > 0) ? delta : 1000;

					let moof = MP4.moof(this.sequenceNo++, [this.track]);
					let mdat = MP4.mdat(sample.data);
					let result = new Uint8Array(moof.byteLength + mdat.byteLength);
					//result.set(MP4.STYP);
					result.set(moof);
					result.set(mdat, moof.byteLength);
					this.doAppend(result);
				
					//this.doAppend();
					//this.doAppend(MP4.mdat(sample.data));
					this.track.samples = [];
					this.track.baseMediaDecodeTime += delta;

					//this.segstream.flush();
					//if (isKeyFrame(pckg[5])) console.log("Key frame ", streampckg[0]);
					/*function decode(value){
						this.converter.appendRawData(value);
					}
					decode(pckg[5]);*/
					//if (this.converter.sourceBuffer && this.converter.sourceBuffer.mode != "sequence") {
					//	this.converter.sourceBuffer.mode = 'sequence';
					//}
					//this.converter.appendRawData(pckg[5], (streampckg[0]-ts));
					//this.converter.play();

					/*if (ts > 0) {
						this.converter.feed({
							video: pckg[5],
							duration: streampckg[0]-ts
						});
					} else {
						this.converter.feed({
							video: pckg[5]
						});
					}*/

					ts = streampckg[0];
				}
				
				
				/*else {
					if (ts > 0) {
						dts = streampckg[0] - ts;
						console.log("Framerate = ", 1000/dts);
						//this.converter = new VideoConverter.default(this.element, 31, 1);
						
						dts = 0;
					}
					ts = streampckg[0];
				}*/
			}
        } else if (pckg[0] === 103) {
			//console.log(msgpack.decode(pckg[5]));
		}
	});
	
	//this.start();
	if (this.peer.status == 2) {
		this.start(0,0,0);
	} else {
		this.peer.on("connect", (p)=> {
			this.start(0,0,0);
		});
	}

	this.element.play();
}

FTLStream.prototype.on = function(name, cb) {
	if (!this.handlers.hasOwnProperty(name)) {
		this.handlers[name] = [];
	}
	this.handlers[name].push(cb);
}

FTLStream.prototype.notify = function (name, ...args) {
	if (this.handlers.hasOwnProperty(name)) {
		let a = this.handlers[name];
		for (let i=0; i<a.length; ++i) {
			a[i].apply(this, args);
		}
	}
}

FTLStream.prototype.pause = function() {
	this.paused = !this.paused;
	if (!this.paused) {
		this.start(0,0,0);
		this.element.play();
	} else {
		this.element.pause();
	}
}

FTLStream.prototype.updatePose = function() {
	let poseRX = rematrix.rotateX(this.rotationX);
	let poseRY = rematrix.rotateY(this.rotationY);
	let poseRZ = rematrix.rotateZ(this.rotationZ);
	let poseT = rematrix.translate3d(this.translateX, this.translateY, this.translateZ);
	let pose = [poseT,poseRX,poseRY,poseRZ].reduce(rematrix.multiply);
	this.setPose(pose);
}

FTLStream.prototype.setPose = function(pose) {
	if (pose.length != 16) {
		console.error("Invalid pose");
		return;
	}
	this.peer.send(this.uri, 0, [1, this.current_fs, this.current_source, 66],
		[103, 7, 1, 0, 0, msgpack.encode(pose)]);
}

FTLStream.prototype.start = function(fs, source, channel) {
	let id = "id-"+fs+"-"+source+"-"+channel;
	this.current = id;
	this.current_fs = fs;
	this.current_source = source;
	this.current_channel = channel;

	if (this.found) {
		this.peer.send(this.uri, 0, [1,fs,255,channel],[255,7,35,0,0,Buffer.alloc(0)]);
	} else {
		this.peer.rpc("find_stream", (res) => {
			this.found = true;
			this.peer.send(this.uri, 0, [1,fs,255,channel],[255,7,35,0,0,Buffer.alloc(0)]);
		}, this.uri);
	}
}


/*connectToStream = () => {
    const element = document.getElementById('ftlab-stream-video');
    let converter = null;

    let rxcount = 0;
    let ts = 0;
    let dts = 0;

    peer.bind(current_data.uri, (latency, streampckg, pckg) => {
        if(pckg[0] === 2){
            rxcount++;
            if (rxcount >= 25) {
                rxcount = 0;
                peer.send(current_data.uri, 0, [1,0,255,0],[255,7,35,0,0,Buffer.alloc(0)]);
                //peer.send(current_data.uri, 0, [255,7,35,0,0,Buffer.alloc(0)], [1,0,255,0]);
            }

            if (converter) {
                function decode(value){
                    converter.appendRawData(value);
                }
                decode(pckg[5]);
                converter.play();
            } else {
                if (ts > 0) {
                    dts = streampckg[0] - ts;
                    console.log("Framerate = ", 1000/dts);
                    converter = new VideoConverter.default(element, 30, 1);
                }
                ts = streampckg[0];
            }
        } else if (pckg[0] === 103) {
			console.log(msgpack.decode(pckg[5]));
		}
    })

    // Start the transaction
    //peer.send("get_stream", (current_data.uri, 30, 0, current_data.uri));

    peer.rpc("find_stream", (res) => {
        peer.send(current_data.uri, 0, [1,0,255,0],[255,7,35,0,0,Buffer.alloc(0)]);
        //peer.send(current_data.uri, [255,7,35,0,0,Buffer.alloc(0)], [1,0,255,0]);
    }, current_data.uri);
}*/

closeStream = () => {
    peer.sock.close()
}



/**
 * **************
 * CONFIGURATIONS
 * **************
 */


current_data.configURI = "ftl://utu.fi#reconstruction_snap8/net"

configs = () => {
    const container = document.getElementById("container");
    container.innerHTML = `<div class="ftlab-configurations"></div>`;
    renderConfigOptions();
}


renderConfigOptions = () => {
    const input = `<p>input1</p><br>ftl://utu.fi#<input type="text">`
    const doc = document.getElementsByClassName('ftlab-configurations')[0];
    doc.innerHTML = input;
}

/**
 * 
 */
loadConfigs = async (str) => {
    const configURI = encodeURIComponent(`ftl://utu.fi#reconstruction_snap8${str}`);
    const uri = encodeURIComponent(current_data.uri)
    const rawResp = await fetch(`./stream/config?settings=${configURI}&uri=${uri}`)
    const response = await rawResp.json();
    const content = JSON.parse(response);
    container.innerHTML += `<p>${response}</p>`;
}

// current_data.configData = '{"peers": 1}';

/**
 * Method to send configurations to backend 
 */
saveConfigs = async () => {
    let {uri, configURI, configData} = current_data
    const rawResp = await fetch('./stream/config', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({peerURI: uri, configURI, data: configData, saveToCPP: true})
    });
    const content = await rawResp.json();
}