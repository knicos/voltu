import H264Remuxer from './h264-remuxer';
import NALU from './util/NALU';
export interface SEIMessage {
    type: number;
}
export default class H264Parser {
    private remuxer;
    private track;
    constructor(remuxer: H264Remuxer);
    private parseSEI(sei);
    private parseSPS(sps);
    private parsePPS(pps);
    parseNAL(unit: NALU): boolean;
    private static skipScalingList(decoder, count);
    private static readSPS(data);
    private static readSEI(data);
    private static readSEIMessage(decoder);
    private static readSEIPayload(decoder, type, size);
}
