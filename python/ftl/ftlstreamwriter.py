import msgpack
import struct

from . import ftltype

from codecs import create_encoder

class FTLStreamWriter:
    def __init__(self, file, version=3):
        self._file = open(file, "wb")
        self._file.write(bytes(ord(c) for c in "FTLF")) # magic
        self._file.write(bytes([version]))              # version
        self._file.write(bytes([0]*64))                 # reserved
        self._packer = msgpack.Packer(strict_types=False, use_bin_type=True)

        self._encoders = {}
        self._channel_count = 0

    def __del__(self):
        self.close()

    def close(self):
        self._file.close()

    def add_raw(self, sp, p):
        if len(sp) != len(ftltype.StreamPacket._fields):
           raise ValueError("invalid StreamPacket")

        if len(p) != len(ftltype.Packet._fields):
            raise ValueError("invalid Packet")

        self._file.write(self._packer.pack((sp, p)))
        self._file.flush()

    def create_encoder(self, source, codec, channel, **kwargs):
        if channel not in ftltype.Channel:
            raise ValueError("unknown channel")

        if not isinstance(source, int):
            raise ValueError("source id must be int")

        if source < 0:
            raise ValueError("source id must be positive")

        encoder = create_encoder(codec, channel, **kwargs)
        self._encoders[(int(source), int(channel))] = encoder
        self._channel_count += 1

    def encode(self, source, timestamp, channel, data):
        if not isinstance(source, int):
            raise ValueError("source id must be int")

        if source < 0:
            raise ValueError("source id must be positive")

        if timestamp < 0:
            raise ValueError("timestamp must be positive")

        if channel not in ftltype.Channel:
            raise ValueError("unknown channel")

        try:
            p = self._encoders[(int(source), int(channel))](data)
        except KeyError:
            raise Exception("no encoder found, create_encoder() has to be" +
                            "called for every source and channel")
        except Exception as ex:
            raise Exception("Encoding error:" + str(ex))

        sp = ftltype.StreamPacket._make((timestamp,
                                         int(source),
                                         int(channel),
                                         self._channel_count))

        self.add_raw(sp, p)
