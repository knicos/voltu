import msgpack
from collections import namedtuple
from . libde265 import Decoder

_packet = namedtuple("Packet", ["codec", "definition", "block_total", "block_number", "flags", "data"])
_stream_packet = namedtuple("StreamPacket", ["timestamp", "streamID", "chanel_count", "channel"])

_definition_t = {
    0 : (),
    1 : (),
    2 : (1080, 1920),
    3 : (720, 1280),
    4 : (),
    5 : (),
    6 : (),
    7 : (),
    8 : ()
}

class FTLStream:
    def __init__(self, file):
        self._file = open(file, "br")
        self._decoders = {}
        self._frames = {}
        
        try:
            magic = self._file.read(5)
            if magic[:4] != bytearray(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")
            
            self._unpacker = msgpack.Unpacker(self._file, raw=True, use_list=False)
            
        except Exception as ex:
            self._file.close()
            raise ex
            
        self._packets_read = 0
    
    def __del__(self):
        self._file.close()
    
    def _read_next(self):
        v1, v2 = self._unpacker.unpack()
        return _stream_packet._make(v1), _packet._make(v2)
    
    def _update_calib(self, sp, p):
        ''' Update calibration '''
        pass
    
    def _update_pose(self, sp, p):
        ''' Update pose '''
        pass
    
    def _decode_frame_hevc(self, sp, p):
        ''' Decode HEVC frame '''
        
        k = (sp.streamID, sp.channel)
        
        if k not in self._decoders:
            self._decoders[k] = Decoder(_definition_t[p.definition])
        
        decoder = self._decoders[k]
        decoder.push_data(p.data)
        decoder.decode()
        img = decoder.get_next_picture()
        
        if img is not None:
            self._frames[k] = img
    
    def read(self):
        '''
        Reads data for until the next timestamp. Returns False if there is no
        more data to read, otherwise returns True.
        '''
        if self._packets_read == 0:
            self._sp, self._p = self._read_next()
            self._packets_read += 1
            
        self._frames = {}
        
        ts = self._sp.timestamp
        ex = None
        
        while self._sp.timestamp == ts:
            try:
                if self._p.codec == 100: # JSON
                    NotImplementedError("json decoding not implemented")

                elif self._p.codec == 101: # CALIBRATION
                    self._update_calib(self._sp, self._p)

                elif self._p.codec == 102: # POSE
                    self._update_pose(self._sp, self._p)

                elif self._p.codec == 3: # HEVC
                    self._decode_frame_hevc(self._sp, self._p)

                else:
                    raise ValueError("unkowno codec %i" % p.codec)
            
            except Exception as e:
                ex = e
            
            try:
                self._sp, self._p = self._read_next()
                self._packets_read += 1
            
            except msgpack.OutOfData:
                return False
            
        if ex is not None:
            raise ex
            
        return True
    
    def get_frames(self):
        ''' Returns all frames '''
        return self._frames
    
    def get_frame(self, source, channel):
        k = (source, channel)
        if k in self._frames:
            return self._frames[k]
        else:
            return None
