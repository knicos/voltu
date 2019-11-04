Python support for `.ftl` files. At the moment, only reading RGB channels
(left/right) supported. Non-parallel decoding of 8 streams has a frame rate
of ~15 fps on i7-9700K.

Required **Python** modules:

 * msgpack 
 * numpy
 * skimage **or** OpenCV 

Required libraries

 * libde265 (available on most Linux distributions)

## Example

Example reads from `input.ftl` and writes to `output.ftl`. Calibration and
pose are copied directly (same method can be used for other channels as well).
The program copies left and right frames of source 0 to new file (and re-encodes
them in JPG) when both frames are available.

```python
import ftl
from ftl import types

reader = ftl.FTLStreamReader("./input.ftl")
writer = ftl.FTLStreamWriter("./output")

source_id = 0
fps = 25
frame_t = int(1000.0/fps)
timestamp_out = 0
timestamp_in = 0

im_left = None
im_right = None

while reader.read():
    channel = reader.get_channel_type()
    timestamp = reader.get_timestamp()
    frame = reader.get_frame()
    
    if reader.get_source_id() != source_id:
        # not interested in this source, skip
        continue
    
    if channel in (types.Channel.Calibration, types.Channel.Configuration):
        # copy calibration and pose (but replace timestamp with new value)
        
        sp, p = reader.get_raw()
        sp = sp._replace(timestamp=timestamp_out)
        writer.add_raw(sp, p)
        continue

    if channel not in (types.Channel.Left, types.Channel.Right):
        # only interested in left and right frame
        continue

    if frame is None:
        # no frame if decoding failed
        continue

    if timestamp_in != timestamp:
        # new timestamp, process available frames

        if not (im_left is None or im_right is None):
            # save frames only if both of them were found for this timestamp
            
            # Note: In this expample channel is re-encoded. If channel content
            # is not modified, lossy channels should be passed directly
            # (using add_raw() in same way as for calibration/pose) instead of 
            # re-encoding them.
            
            writer.add_frame(timestamp_out, 0, types.Channel.Left, 2,
                             types.codec_t.JPG, im_left)
            writer.add_frame(timestamp_out, 0, types.Channel.Right, 2,
                             types.codec_t.JPG, im_right)
        
        
        timestamp_out += frame_t
        timestamp_in = timestamp
        im_left, im_right = None, None

    if channel is types.Channel.Left:
        im_left = frame
    else:
        im_right = frame

```
