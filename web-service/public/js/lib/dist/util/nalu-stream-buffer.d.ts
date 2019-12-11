import NALU from './NALU';
export default class VideoStreamBuffer {
    private buffer;
    clear(): void;
    append(value: ArrayLike<number>): NALU[];
    private mergeBuffer(value);
}
