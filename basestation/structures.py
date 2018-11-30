from collections import namedtuple
import enum
import struct
import time

class Orientation(enum.Enum):
    UNKNOWN = 0
    UPRIGHT = 1
    INVERTED = 2
    LEFT = 3
    RIGHT = 4
    TOP = 5
    BOTTOM = 6


class AutoRepr(object):
    def __repr__(self):
        items = ("%s = %r" % (k, v) for k, v in self.__dict__.items())
        return "<%s: {%s}>" % (self.__class__.__name__, ', '.join(items))


IMUSample = namedtuple('IMUSample', 'ax ay az wx wy dt')


class RadioPacket(AutoRepr):
    VERSION = 2

    @classmethod
    def from_bytes(cls, buf: bytes, rssi: float = None, timestamp: float = None):
        packet_format = '<BHHIBHffffB'
        packet_minsize = struct.calcsize(packet_format)

        assert buf, "packet is empty"
        assert buf[0] == cls.VERSION, "packet is not the right version"
        assert len(buf) >= packet_minsize, "packet is too small"

        v, s, t, i, o, c, ax, ay, d, b, ns = struct.unpack(packet_format, buf[:packet_minsize])

        self = cls()
        self.raw = buf
        self.serial = s
        self.tag = t
        self.seq = i
        self.orientation = Orientation(o)
        self.chain = c
        self.angle_x = ax
        self.angle_y = ay
        self.displacement = d
        self.vbatt = b
        self.rssi = rssi

        self.timestamp = timestamp or time.time()
        self.samples = []

        # typedef struct {
        #     double ax, ay, az, wx, wy;
        #     uint16_t dt_us; // shifted >> 2 for a max of ~0.262s
        # } imu_sample_t;

        sample_format = '<fffffH'
        sample_size = struct.calcsize(sample_format)

        for i in range(ns):
            offset = packet_minsize + i*sample_size
            sample_buf = buf[offset : offset+sample_size]
            ax, ay, az, wx, wy, dt = struct.unpack('<fffffH', sample_buf)
            dt = (dt << 2) / 1000000  # convert to seconds
            self.samples.append(IMUSample(ax, ay, az, wx, wy, dt))

        return self
