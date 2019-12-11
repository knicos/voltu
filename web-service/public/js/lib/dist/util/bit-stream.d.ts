export default class BitStream {
    private data;
    private index;
    private bitLength;
    constructor(data: Uint8Array);
    readonly bitsAvailable: number;
    skipBits(size: number): void;
    readBits(size: number): number;
    private getBits(size, offsetBits, moveIndex?);
    skipLZ(): number;
    skipUEG(): void;
    skipEG(): void;
    readUEG(): number;
    readEG(): number;
    readBoolean(): boolean;
    readUByte(): number;
    readUShort(): number;
    readUInt(): number;
}
