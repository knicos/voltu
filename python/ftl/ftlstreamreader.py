import msgpack
import numpy as np

from warnings import warn

from . import ftltype
from . codecs import create_decoder

class FTLStreamReader:
    """ FTL file reader. """

    def __init__(self, file):
        self._file = open(file, "br")
        self._version = 0

        self._decoders = {}
        self._enabled_sources = []
        self._available_sources = []

        self._data = None

        try:
            magic = self._file.read(5)
            self._version = int(magic[4])
            if magic[:4] != bytes(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")

            if self._version >= 2:
                # first 64 bytes reserved
                self._file.read(8*8)

            self._unpacker = msgpack.Unpacker(self._file, raw=True, use_list=False)

        except Exception as ex:
            self._file.close()
            raise ex

        self._packets_read = 0

    def __del__(self):
        self._file.close()

    def _read_next(self):
        v1, v2 = self._unpacker.unpack()
        return ftltype.StreamPacket._make(v1), ftltype.Packet._make(v2)

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
        self._data = None

        try:
            self._sp, self._p = self._read_next()
            self._packets_read += 1

        except msgpack.OutOfData:
            return False

        if self._sp.streamID not in self._available_sources:
            self._available_sources.append(self._sp.streamID)

        if self._enabled_sources and self._sp.streamID not in self._enabled_sources:
            return True

        k = (self._sp.streamID, self._sp.channel)
        if k not in self._decoders:
            self._decoders[k] = create_decoder(self._p.codec, self._sp.channel, self._version)

        self._data = self._decoders[k](self._p)

        return True

    def get_raw(self):
        """ Returns previously read StreamPacket and Packet """
        return self._sp, self._p

    def get_channel(self):
        return ftltype.Channel(self._sp.channel)

    def get_source_id(self):
        return self._sp.streamID

    def get_timestamp(self):
        return self._sp.timestamp

    def get_data(self):
        """ Returns decoded data """
        return self._data

    def get_sources(self):
        """ Return list of sources. Can change as stream is read. """
        return list(self._available_sources)

    def get_version(self):
        return self._version

    def get_packet_count(self):
        return self._packets_read
