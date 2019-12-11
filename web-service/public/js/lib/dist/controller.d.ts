export declare const mimeType = "video/mp4; codecs=\"avc1.42E01E\"";
export declare class VideoController {
    private element;
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
    constructor(element: HTMLVideoElement);
    setup(): Promise<void>;
    play(): void;
    pause(): void;
    reset(): void;
    appendRawData(data: ArrayLike<number>): void;
    private writeFragment(dts, pay);
    private writeBuffer(data);
    private doAppend(data);
}
