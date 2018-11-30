#!/usr/bin/env python3

import enum
from math import log10
import struct
from time import sleep
import time
import threading
import traceback

from gpiozero import Button, LED, RGBLED
import RPi.GPIO as GPIO

try:
    import rf95
except ImportError:
    rf95 = None


## BEGIN VARIABLE DECLARATIONS ##

RF95_SPI_CS = 0
RF95_IRQ_PIN = 12
RF95_RESET_PIN = 21
RF95_FREQ = 915.0

SEGMENTS = {
    # val:  A,B,C,D,E,F,G
    None : (0,0,0,0,0,0,0),
    '-'  : (0,0,0,0,0,0,1),
    0    : (1,1,1,1,1,1,0),
    1    : (0,1,1,0,0,0,0),
    2    : (1,1,0,1,1,0,1),
    3    : (1,1,1,1,0,0,1),
    4    : (0,1,1,0,0,1,1),
    5    : (1,0,1,1,0,1,1),
    6    : (1,0,1,1,1,1,1),
    7    : (1,1,1,0,0,0,0),
    8    : (1,1,1,1,1,1,1),
    9    : (1,1,1,1,0,1,1)
}

# Common Cathode: active segment pins must be high and digit pin low
SEGMENT_PINS = (24, 25, 26, 13, 6, 5, 23, 19)  # A, B, C, D, E, F, G, dot
DIGIT_PINS = (22, 27)  # tens, ones
RGB_PINS = (16, 20, 4)  # R, G, B

GPIO.setup(SEGMENT_PINS + DIGIT_PINS, GPIO.OUT)

## END OF VARIABLE DECLARATIONS ##

## BEGIN DATA STRUCTURES ##

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


class RadioPacketv1(AutoRepr):
    @classmethod
    def from_bytes(cls, buf: bytes, rssi: float = None):
        assert len(buf) and buf[0] == 1
        v, s, t, i, o, c, ax, ay, d, b = struct.unpack('<BIIIBIffff', buf)
        self = cls()
        self.version = v
        self.serial = s
        self.tag = t
        self.seq = i
        self.orientation = Orientation(o)
        self.chain = c
        self.angle_x = ax
        self.angle_y = ay
        self.displacement = d
        self.vbatt = b
        return self

## END DATA STRUCTURES ##

## BEGIN APPLICATION STRUCTURES ##

class LCDLogic:
    def __init__(self, segment_pins: tuple, digit_pins: tuple):
        self.digit_pins = digit_pins
        self.segment_pins = segment_pins
        self.num_digits = len(digit_pins) or 1

    def parse_number(self, number: int, num_digits: int = None, pad_zeros: bool = False) -> list:
        "Returns a list of digits, with the 0 index as the ones place, 1 as tens, etc."
        if num_digits is None:
            num_digits = self.num_digits

        ret = [0 if pad_zeros else None] * num_digits

        if number == 0:
            ret[0] = number
        else:
            for i in range(num_digits):
                ret[i] = number % 10
                number //= 10

                if not number:
                    break

        return ret

    def pin_values(self, number: int = None, pad_zeros: bool = False) -> list:
        if number is None:
            return [SEGMENTS[None]] * self.num_digits

        max_disp = 10 ** (self.num_digits - 1)
        abs_number = abs(number)

        # handle decimal and negative values
        if (abs_number < max_disp and int(abs_number) != abs_number) or number < 0:
            pass  # TODO
            places = [None] * self.num_digits
        else:
            places = self.parse_number(number, pad_zeros=pad_zeros)

        return [SEGMENTS[x] for x in places]

    def pins_values(self, number: int = None, pad_zeros: bool = False) -> list:
        def_segment_values = [1] * len(self.digit_pins)
        pins = self.segment_pins + self.digit_pins
        cycles = []

        for i, segment in enumerate(self.pin_values(number, pad_zeros)):
            l = list(segment)
            l.append(0)  # dot
            l.extend(def_segment_values)
            l[-1 - i] = 0
            cycles.append(l)

        return [(pins, c) for c in cycles]


class LCDThread(threading.Thread):
    def __init__(self, segment_pins, digit_pins):
        threading.Thread.__init__(self)
        self.stop_flag = False
        self.logic = LCDLogic(segment_pins, digit_pins)
        self.pins_values = self.logic.pins_values(None)

    def set_value(self, num: int, pad_zeros: bool = False):
        self.pins_values = self.logic.pins_values(num, pad_zeros)

    def clear(self):
        self.pins_values = self.logic.pins_values(None)

    def stop(self):
        self.stop_flag = True
        self.join()
        p, v = self.logic.pins_values(None)[0]
        GPIO.output(p, v)

    def run(self):
        while not self.stop_flag:
            for pins, values in self.pins_values:
                GPIO.output(pins, values)
                sleep(0.01)


class AlertHarness:
    def __init__(self, test: bool = False):
        if test:
            self.radio = None
        else:
            r = self.radio = rf95.RF95(cs=RF95_SPI_CS, int_pin=RF95_IRQ_PIN, reset_pin=RF95_RESET_PIN)
            assert r.init(), "failed to intialize radio"
            r.set_frequency(RF95_FREQ)

        self.test = test
        self.led = RGBLED(*RGB_PINS)

        lt = self.lcd_thread = LCDThread(SEGMENT_PINS, DIGIT_PINS)
        lt.start()

        self.trouble_tags = set()

    def get_packet(self, timeout=None) -> RadioPacketv1:
        if self.test:
            return

        start = time.time()

        while not self.radio.available():
            if timeout and (time.time() - start > timeout):
                return None

        rxbuf = self.radio.recv()
        rxbuf = bytes(rxbuf[4:])  # skip flag
        rssi = self.radio.last_rssi

        try:
            return RadioPacketv1.from_bytes(rxbuf, rssi)
        except Exception as e:
            h = rxbuf.hex()
            hexstr = ' '.join(h[x:x+2] for x in range(0, len(h), 2))
            new_e = RuntimeError("error parsing packet with length %d and data:\n%s" % (len(rxbuf), hexstr))
            raise new_e from e

    def run_test(self):
        print("Test mode: will simulate an alert in 10 seconds")
        sleep(10)
        self.show_alert(42)
        print("Test mode: returning to normal in 5 seconds")
        sleep(5)
        self.clear_alert()
        print("Test mode: will exit in 5 seconds")
        sleep(5)

    def run(self):
        # Initialize everything as OK
        self.clear_alert()

        if self.test:
            return self.run_test()

        while True:
            try:
                packet = self.get_packet()

                if not packet:
                    continue
                elif packet.chain < 5:  # ignore unless condition has lasted 5 cycles
                    continue

                if packet.orientation is not Orientation.UPRIGHT and packet.tag not in self.trouble_tags:
                    self.trouble_tags.add(packet.tag)
                    self.show_alert(packet.tag)
                elif packet.orientation is Orientation.UPRIGHT and packet.tag in self.trouble_tags:
                    self.trouble_tags.remove(packet.tag)
                    self.clear_alert()

            except Exception as e:
                traceback.print_exc()

    def show_alert(self, tag: int):
        print("showing alert for tag", tag)
        self.lcd_thread.set_value(tag)
        self.led.pulse(fade_in_time = 1, fade_out_time = 1, on_color = (1,0,0), background = True)

    def clear_alert(self):
        print("clearing alert")
        self.led.blink(on_time = 0.1, off_time = 5 - 0.1, on_color = (0,1,0), background = True)
        self.lcd_thread.clear()

    def shutdown(self):
        self.led.close()
        self.lcd_thread.stop()

        if self.radio:
            self.radio.cleanup()


if __name__ == "__main__":
    if rf95 is None:
        testing = True
        print("Radio lib unavailable, entering test mode")
    else:
        testing = False

    harness = AlertHarness(test=testing)

    try:
        harness.run()
    except KeyboardInterrupt:
        pass
    finally:
        harness.shutdown()
