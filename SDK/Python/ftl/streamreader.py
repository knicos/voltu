import msgpack
import numpy as np

from warnings import warn

from . types import Channel, Camera
from . codecs import create_decoder, StreamPacket, Packet, PacketFlags

class FTLStreamReader:
    """ FTL file reader. """

    def __init__(self, file, max_buffer_size=64*2**20):
        self._file = open(file, "br")
        self._version = 0

        self._decoders = {}
        self._available_sources = []

        self._p = None
        self._sp = None
        self._decoded = None
        self._frames = []

        try:
            magic = self._file.read(5)
            self._version = int(magic[4])
            if magic[:4] != bytes(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")

            if self._version >= 2:
                # first 64 bytes reserved
                self._file.read(8*8)

            self._unpacker = msgpack.Unpacker(self._file, raw=True,
                                              use_list=False,
                                              max_buffer_size=max_buffer_size)

        except Exception as ex:
            self._file.close()
            raise ex

        self._packets_read = 0

    def __del__(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self._file.close()

    def __iter__(self):
        return self

    def __next__(self):
        if len(self._frames) > 0:
            return self._frames.pop(0)

        if not self.read():
            raise StopIteration

        self._frames = self.get()
        return self._frames.pop(0)

    def _read_next(self):
        v1, v2 = self._unpacker.unpack()
        return StreamPacket._make(v1), Packet._make(v2)

    def seek(self, ts):
        """ Read until timestamp reached """
        if self.get_timestamp() >= ts:
            raise Exception("trying to seek to earlier timestamp")

        while self.read():
            if self.get_timestamp() >= ts:
                break

    def read(self):
        """
        Reads data for until the next timestamp. Returns False if there is no
        more data to read, otherwise returns True.
        """
        self._decoded = None

        try:
            self._sp, self._p = self._read_next()
            self._packets_read += 1

        except msgpack.OutOfData:
            return False

        if self._sp.frameset_id not in self._available_sources:
            self._available_sources.append(self._sp.frameset_id)

        k = (self._sp.frameset_id, self._sp.channel)
        if k not in self._decoders:
            self._decoders[k] = create_decoder(self._p.codec, self._sp.channel, self._version)

        self._decoded = self._decoders[k](self._p)
        return True

    def get(self):
        if isinstance(self._decoded, list):
            if self._p.flags == PacketFlags.Partial:
                raise NotImplementedError("partial packets not implemented (todo)")

            res = []
            for src, data in enumerate(self._decoded):
                # How are sources and frames mapped? What if frame missing from
                # one of the sources?
                res.append((self.get_timestamp(), src, Channel(self._sp.channel), data))

            return res

        else:
            return [(self.get_timestamp(), self._sp.frame_number, Channel(self._sp.channel), self._decoded)]

    def get_raw(self):
        """ Returns previously read StreamPacket and Packet """
        return self._sp, self._p

    def get_timestamp(self):
        return self._sp.timestamp

    def get_sources(self):
        """ Return list of sources. """
        return list(self._available_sources)

    def get_version(self):
        return self._version

    def get_packet_count(self):
        return self._packets_read
