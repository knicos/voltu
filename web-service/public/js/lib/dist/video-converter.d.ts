export declare const mimeType = "video/mp4; codecs=\"avc1.42E01E\"";
export default class VideoConverter {
    private element;
    private fps;
    private fpf;
    private mediaSource;
    private sourceBuffer;
    private receiveBuffer;
    private remuxer;
    private mediaReady;
    private mediaReadyPromise;
    private queue;
    private isFirstFrame;
    static readonly errorNotes: {
        [x: number]: string;
    };
    constructor(element: HTMLVideoElement, fps?: number, fpf?: number);
    private setup();
    play(): void;
    pause(): void;
    reset(): void;
    appendRawData(data: ArrayLike<number>): void;
    private writeFragment(dts, pay);
    private writeBuffer(data);
    private doAppend(data);
}
