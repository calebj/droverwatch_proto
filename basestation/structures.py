from collections import Counter, defaultdict, namedtuple, OrderedDict, deque
import enum
import struct
import time
from typing import Sequence


ROLL_ADJUST = 180
WALKING_THRESH = 0.6
RUNNING_THRESH = 2
OR_RATIO = 0.4
WINDOW_WIDTH = 7
WINDOW_CUTOFF = 10


def shift_roll(roll: float) -> float:
    rot = (roll + ROLL_ADJUST) % 360
    return (-360 + rot) if rot > 180 else rot


def weighted_avg(points: Sequence[float], bias=0.5) -> float:
    num_points = len(points)

    if num_points == 1:
        return points[0]

    first_y = 1 - bias
    last_y = 1 + bias
    dy = (last_y - first_y) / (num_points - 1)
    return sum(((first_y + i * dy) * x) for i, x in enumerate(points)) / num_points


class Orientation(enum.Enum):
    UNKNOWN = 0
    UPRIGHT = 1
    INVERTED = 2
    LEFT = 3
    RIGHT = 4
    TOP = 5
    BOTTOM = 6

    @classmethod
    def resolve(cls, angle_x: float, angle_y: float) -> "Orientation":
        if abs(angle_x) < 45:
            if abs(angle_y) < 45:
                return cls.UPRIGHT
            elif abs(angle_y) > 135:
                return cls.INVERTED
            elif angle_y > 45:
                return cls.BOTTOM
            elif angle_y < -45:
                return cls.TOP
        elif angle_x < -45:
            return cls.RIGHT
        elif angle_x > 45:
            return cls.LEFT
        else:
            return cls.UNKNOWN


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

        ay = shift_roll(ay)

        self = cls()
        self.raw = buf
        self.serial = s
        self.tag = t
        self.seq = i
        # self.orientation = Orientation(o)
        self.orientation = Orientation.resolve(ax, ay)
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
            offset = packet_minsize + i * sample_size
            sample_buf = buf[offset : offset + sample_size]
            ax, ay, az, wx, wy, dt = struct.unpack('<fffffH', sample_buf)
            dt = (dt << 2) / 1000000  # convert to seconds
            self.samples.append(IMUSample(ax, ay, az, wx, wy, dt))

        return self


class State(enum.Enum):  # TODO: use IntFlag (requires python 3.6+)
    UNKNOWN = 0
    IDLE = 1
    WALKING = 3
    RUNNING = 5
    DISTRESS = 10
    MISSING = 30

    @classmethod
    def resolve(cls, packets):
        cutoff = time.time() - WINDOW_CUTOFF
        orig_packets = packets

        if not packets:
            return cls.UNKNOWN
        elif orig_packets[-1].timestamp < cutoff:
            return cls.MISSING

        packets = [p for p in orig_packets if p.timestamp > cutoff]

        or_counter = Counter(p.orientation for p in packets)
        orientation, or_count = or_counter.most_common(1)[0]

        if or_count < len(packets) * OR_RATIO:
            orientation = Orientation.UNKNOWN

        motion_samples = [p.displacement for p in packets]
        avg_motion = weighted_avg(motion_samples, bias=0.75)

        if orientation in (Orientation.LEFT, Orientation.RIGHT):
            return cls.DISTRESS
        elif avg_motion > RUNNING_THRESH:
            return cls.RUNNING
        elif avg_motion > WALKING_THRESH:
            return cls.WALKING
        else:
            return cls.IDLE


class StateTracker:
    def __init__(self):
        self._last_packets = defaultdict(lambda: deque(maxlen=WINDOW_WIDTH))

    def state_for(self, key) -> State:
        return State.resolve(self._last_packets[key])

    def update(self, packet: RadioPacket):
        self._last_packets[packet.tag].append(packet)
