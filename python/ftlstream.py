import msgpack
from collections import namedtuple

_packet = namedtuple("Packet", ["codec", "definition", "block_total", "block_number", "flags", "data"])
_stream_packet = namedtuple("StreamPacket", ["timestamp", "streamID", "chanel_count", "channel"])

class FTLStream:
    def __init__(self, file):
        self._file = open(file, "br")
        
        try:
            magic = self._file.read(5)
            if magic[:4] != bytearray(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")
            
            self._unpacker = msgpack.Unpacker(self._file, raw=True, use_list=False)
            
        except Exception as ex:
            self._file.close()
            raise ex
    
    def __del__(self):
        self._file.close()
    
    def read(self):
        # TODO: Different methods for reading different types?
        return self._read_next()
    
    def _read_next(self):
        v1, v2 = self._unpacker.unpack()
        return _stream_packet._make(v1), _packet._make(v2)