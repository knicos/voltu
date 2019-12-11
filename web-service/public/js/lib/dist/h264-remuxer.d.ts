import { Track } from './types';
import NALU from './util/NALU';
export default class H264Remuxer {
    fps: number;
    framePerFragment: number;
    timescale: number;
    readyToDecode: boolean;
    private totalDTS;
    private stepDTS;
    private frameCount;
    private seq;
    mp4track: Track;
    private unitSamples;
    private parser;
    private static getTrackID();
    constructor(fps: number, framePerFragment: number, timescale: number);
    readonly seqNum: number;
    remux(nalu: NALU): [number, Uint8Array] | undefined;
    private createNextFrame();
    flush(): void;
    private getFragment();
    private checkReadyToDecode();
}
