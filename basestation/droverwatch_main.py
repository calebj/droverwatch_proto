#!/usr/bin/env python3

import argparse
from multiprocessing.connection import Client as mpClient
import os
import pathlib
import signal
import subprocess
from time import sleep
import time
import threading
import traceback
from typing import Optional

from gpiozero import RGBLED
import RPi.GPIO as GPIO

from structures import Orientation, RadioPacket, StateTracker, State

try:
    import twilio
    from twilio.rest import Client as TwilioClient
except ImportError:
    print("Twilio not available")
    twilio = None

try:
    import sendgrid
    from sendgrid.helpers.mail import Content, Email, Mail, Personalization
except ImportError:
    print("Sendgrid not available")
    sendgrid = None

try:
    from pyRF95 import rf95
except ImportError:
    rf95 = None

CWD = pathlib.Path(os.path.dirname(os.path.realpath(__file__)))

## BEGIN VARIABLE DECLARATIONS ##

ALARM_SOUND = str(CWD / "BaseAlarmLoop_clip.wav")
TTS_SCRIPT = str(CWD / "say.sh")

PLT_DEFAULT_PORT = 56700
ACTIVE_SLEEP = 2

TWILIO_SID = "..."
TWILIO_AUTH  = "..."
TWILIO_NUMBER = "+1..."

SENDGRID_API_KEY = "..."
SENDGRID_FROM_ADDRESS = "alerts@yourdomain.com"

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
DIGIT_PINS = (27, 22)  # tens, ones
RGBLED_PINS = (4, 20, 16)

RED = (1, 0, 0)
GREEN = (0, 1, 0)
YELLOW = (1, 1, 0)

GPIO.setup(SEGMENT_PINS + DIGIT_PINS, GPIO.OUT)

## END OF VARIABLE DECLARATIONS ##

## BEGIN DATA STRUCTURES ##

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
            pin_values = list(segment)
            pin_values.append(0)  # dot
            pin_values.extend(def_segment_values)
            pin_values[-1 - i] = 0
            cycles.append(pin_values)

        return [(pins, c) for c in cycles]


class BackgroundThread(threading.Thread):
    def __init__(self, callback, *args, start=True, **kwargs):
        threading.Thread.__init__(self)
        assert callable(callback)
        self.callback = callback
        self.args = args
        self.kwargs = kwargs

        if start:
            self.start()

    def run(self):
        self.callback(*self.args, **self.kwargs)


class CommandThread(threading.Thread):
    def __init__(self, cmd, callback=None, start=True):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.callback = callback
        self.proc = None
        self.result = None
        self.stop_flag = False

        if start:
            self.start()

    def stop(self):
        self.stop_flag = True
        self.join()

    def run(self):
        self.proc = subprocess.Popen(self.cmd, preexec_fn=os.setsid)

        while self.proc.returncode is None and not self.stop_flag:
            try:
                self.result = self.proc.wait(0.05)
            except subprocess.TimeoutExpired:
                pass

        if self.result is None:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            self.proc.wait()

        if self.callback:
            self.callback(self.result)


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
                sleep(0.005)


class AlertHarness:
    def __init__(self, args):
        if args.test or args.replay:
            self.radio = None
        else:
            r = self.radio = rf95.RF95(cs=RF95_SPI_CS, int_pin=RF95_IRQ_PIN, reset_pin=RF95_RESET_PIN)
            assert r.init(), "failed to intialize radio"
            r.set_modem_config(rf95.Bw125Cr45Sf128)
            r.set_frequency(RF95_FREQ)

        self._args = args

        self.log_handle = open(args.log, 'w') if args.log else None
        self.replay_handle = open(args.replay, 'r') if args.replay else None
        self._pending_line = None
        self._time_offset = 0

        if self.replay_handle:
            if self.read_replay_line():
                self._time_offset = time.time() - self._pending_line[0]

        self.test = args.test
        self.use_tts = args.tts
        self.use_alarm = args.alarm
        self.numbers = args.phone
        self.emails = args.email

        self.stopping = False
        self.led = RGBLED(*RGBLED_PINS)

        self.lcd_thread = LCDThread(SEGMENT_PINS, DIGIT_PINS)

        self.trouble_tags = set()
        self.known_tags = set()

        self.monitor_thread = BackgroundThread(self.status_loop, start=False)
        self.announce_thread = None
        self._buzzer = False

        if twilio is not None:
            self.twilio_client = TwilioClient(TWILIO_SID, TWILIO_AUTH)
        else:
            self.twilio_client = None

        if sendgrid is not None:
            self.mail_client = sendgrid.SendGridAPIClient(apikey=SENDGRID_API_KEY)
        else:
            self.mail_client = None

        self.conn = None
        self.states = StateTracker()

    def read_replay_line(self) -> bool:
        try:
            line = next(self.replay_handle)
            ts, rssi, pkt_hex = line.split(",")
            ts = float(ts.strip())
            rssi = float(rssi.strip())
            pkt = bytes.fromhex(pkt_hex.strip())
            self._pending_line = ts, rssi, pkt
            return True
        except (StopIteration, ValueError):
            self._pending_line = None
            return False

    def set_plotter(self, address: str, port: int = PLT_DEFAULT_PORT):
        if isinstance(port, str):
            port = int(port)
        elif port is None:
            port = PLT_DEFAULT_PORT

        self.conn = mpClient((address, port))

    def send_sms(self, body: str):
        if self.twilio_client is None or not self.numbers:
            return False

        ret = []
        for number in self.numbers:
            msg = self.twilio_client.messages.create(to=number, from_=TWILIO_NUMBER, body=body)
            ret.append(msg)

        return ret

    def send_email(self, subject: str, body: str):
        if self.mail_client is None or not self.emails:
            return False

        personalization = Personalization()

        for email in self.emails:
            email = Email(email)
            personalization.add_to(email)

        from_email = Email(SENDGRID_FROM_ADDRESS)
        content = Content("text/plain", body)
        mail = Mail(from_email, subject, None, content)
        mail.add_personalization(personalization)
        return self.mail_client.client.mail.send.post(request_body=mail.get())

    def get_packet(self, timeout=None) -> Optional[RadioPacket]:
        packet = None

        if self.test:
            return
        elif self.replay_handle:
            packet = self.replay_packet(timeout)
        elif self.radio:
            start = time.time()

            while not self.radio.available():
                sleep(0.005)
                if timeout and (time.time() - start > timeout):
                    return

            rxbuf = self.radio.recv()
            rxbuf = bytes(rxbuf[4:])  # skip flag
            rssi = self.radio.last_rssi

            try:
                packet = RadioPacket.from_bytes(rxbuf, rssi, timestamp=time.time())
            except Exception as e:
                h = rxbuf.hex()
                hexstr = ' '.join(h[x:x + 2] for x in range(0, len(h), 2))
                new_e = RuntimeError("error parsing packet with length %d and data:\n%s"
                                     % (len(rxbuf), hexstr))
                raise new_e from e

        if packet:
            self.states.update(packet)
            return packet

    def replay_packet(self, timeout=None):
        if self._pending_line:
            target = self._pending_line[0] + self._time_offset
            sleep_for = target - time.time()

            if timeout and sleep_for > timeout:
                sleep(timeout)
                return None
            else:
                sleep(max(0, sleep_for))
                timestamp, rssi, rxbuf = self._pending_line
                timestamp = target

                if not self.read_replay_line():
                    print("end of log, stopping")
                    self.stopping = True

                return RadioPacket.from_bytes(rxbuf, rssi, timestamp=timestamp)

    def run_test(self):
        print("Test mode: will simulate an alert in 10 seconds")
        sleep(10)
        self.show_alert(42)
        print("Test mode: returning to normal in 10 seconds")
        sleep(10)
        self.clear_alert()
        print("Test mode: will exit in 5 seconds")
        sleep(5)

    def run(self):
        # Initialize everything as OK
        self.clear_alert()
        self.monitor_thread.start()
        self.lcd_thread.start()

        if self.test:
            return self.run_test()

        while not self.stopping:
            try:
                packet = self.get_packet(timeout=1)

                if not packet:
                    continue

                if self.conn:
                    self.conn.send((packet.raw, packet.rssi, packet.timestamp))

                if self.log_handle:
                    line = '%s,%s,%s' % (packet.timestamp, packet.rssi, packet.raw.hex())
                    self.log_handle.write(line + '\n')

                # print(packet.serial, packet.seq, packet.orientation, packet.rssi, packet.vbatt)
                # pprint(packet.samples[-1])

                self.known_tags.add(packet.tag)

                if self.states.state_for(packet.tag) is State.DISTRESS:
                    if packet.tag not in self.trouble_tags:
                        self.trouble_tags.add(packet.tag)
                        self.show_alert(packet.tag)
                elif packet.tag in self.trouble_tags:
                    self.trouble_tags.remove(packet.tag)
                    self.clear_alert()

            except Exception:
                traceback.print_exc()

    def status_loop(self):
        while not self.stopping:
            for i in range(int(ACTIVE_SLEEP / 0.01)):
                sleep(0.01)

                if self.stopping:
                    break

            tag_strs = ['%d: %s' % (t, self.states.state_for(t).name.lower())
                        for t in self.known_tags]
            print("Active tags:\n%s" % ('\n'.join(tag_strs) or '(none)'))

    def show_alert(self, tag: int):
        print("showing alert for tag", tag)
        self.lcd_thread.set_value(tag)
        self.led.pulse(on_color=RED)

        msg = "Tag %s is in distress!" % tag
        self.announce_msg = msg
        self.announce()

        self.send_email("Bovine Intervention alert", msg)
        self.send_sms("Bovine Intervention alert: " + msg)

    def clear_alert(self):
        print("clearing alert")
        self.led.blink(on_color=GREEN, on_time=0.05, off_time=5 - 0.05)
        self.lcd_thread.clear()
        self.announce_msg = None

        if self.announce_thread:
            self.announce_thread.stop()  # block

    def announce_cb(self, result):
        if self.announce_msg is None or self.stopping:
            # reset to None
            self.announce_thread = None
            self._buzzer = False
        elif self._buzzer:
            self._buzzer = False
            self.announce_thread = CommandThread([TTS_SCRIPT, self.announce_msg], self.announce_cb)
        else:
            self._buzzer = True
            self.announce_thread = CommandThread(["mplayer", "-really-quiet", ALARM_SOUND], self.announce_cb)

    def announce(self):
        if self.announce_thread:
            self.announce_thread.join()  # block

        self.announce_thread = CommandThread(["mplayer", ALARM_SOUND], self.announce_cb)
        self._buzzer = True

    def shutdown(self):
        self.stopping = True
        self.lcd_thread.stop()
        self.monitor_thread.join()
        self.led.close()
        #GPIO.cleanup()

        if self.announce_thread:
            self.announce_thread.stop()

        if self.radio:
            self.radio.cleanup()

        if self.log_handle:
            self.log_handle.close()

        if self.replay_handle:
            self.replay_handle.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DroverWatch base station")
    parser.add_argument("-n", "--phone", help="Phone to send alerts to", action="append")
    parser.add_argument("-e", "--email", help="Email to send alerts to", action="append")
    parser.add_argument("-t", "--test", help="Test mode", action="store_true")
    parser.add_argument("--alarm", help="Use sound alarm", action="store_true")
    parser.add_argument("--tts", help="Use TTS", action="store_true")
    parser.add_argument("-p", "--plotter", help="plotter address[:port] (default 56700)")
    parser.add_argument("-l", "--log", metavar="FILE")
    parser.add_argument("-r", "--replay", metavar="FILE")

    args = parser.parse_args()

    if rf95 is None and not args.test:
        args.test = True
        print("Radio lib unavailable, entering test mode")

    if args.log:
        assert not args.replay, "cannot log and replay simultaneously"
        assert not args.test, "cannot log in test mode"

    print("Initializing...")
    harness = AlertHarness(args)

    if args.plotter:
        print("Connecting to plotter...")
        plt_args = args.plotter.split(':')
        harness.set_plotter(*plt_args[:2])

    try:
        print("Started.")
        harness.run()
    except KeyboardInterrupt:
        pass
    finally:
        harness.shutdown()
