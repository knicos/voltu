export default class NALU {
    data: Uint8Array;
    nri: number;
    ntype: number;
    static readonly NDR: number;
    static readonly IDR: number;
    static readonly SEI: number;
    static readonly SPS: number;
    static readonly PPS: number;
    static readonly TYPES: {
        [x: number]: string;
    };
    static type(nalu: NALU): string;
    constructor(data: Uint8Array);
    type(): number;
    isKeyframe(): boolean;
    getSize(): number;
    getData(): Uint8Array;
}
