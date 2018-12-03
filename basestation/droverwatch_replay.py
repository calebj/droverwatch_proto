#!/usr/bin/env python3

import argparse
# from math import log10
from multiprocessing.connection import Client as mpClient
import os
# from pprint import pprint
import signal
import subprocess
from time import sleep
import time
import threading
import traceback
from typing import Optional

from structures import Orientation, RadioPacket, StateTracker  # , IMUSample

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

## BEGIN VARIABLE DECLARATIONS ##

PLT_DEFAULT_PORT = 56700
ACTIVE_SLEEP = 2

TWILIO_SID = "ACe3e295a5969e040e0ca29e0061c8ee2f"
TWILIO_AUTH  = "03820cb03da55ec7024e87f8336b6452"
TWILIO_NUMBER = "+13462585671"

SENDGRID_API_KEY = "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"
SENDGRID_FROM_ADDRESS = "alerts@bovineintervention.com"

## END OF VARIABLE DECLARATIONS ##

## BEGIN DATA STRUCTURES ##

## END DATA STRUCTURES ##

## BEGIN APPLICATION STRUCTURES ##

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
                self.result = self.proc.wait(0.01)
            except subprocess.TimeoutExpired:
                pass

        if self.result is None:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            self.proc.wait()

        if self.callback:
            self.callback(self.result)


class AlertHarness:
    def __init__(self, args):
        self.radio = None

        self._args = args

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

        self.trouble_tags = set()
        self.known_tags = set()

        self.monitor_thread = BackgroundThread(self.status_loop, start=False)
        self.announce_thread = None

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
            personalization.add_to(email)

        from_email = Email(SENDGRID_FROM_ADDRESS)
        content = Content("text/plain", body)
        mail = Mail(from_email, subject, None, content)
        mail.add_personalization(personalization)
        return self.mail_client.client.mail.send.post(request_body=mail.get())

    def get_packet(self, timeout=None) -> Optional[RadioPacket]:
        packet = None

        if self.replay_handle:
            packet = self.replay_packet(timeout)

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
        sleep(1)
        self.show_alert(42)
        print("Test mode: returning to normal in 5 seconds")
        sleep(1)
        self.clear_alert()
        print("Test mode: will exit in 5 seconds")
        sleep(5)

    def run(self):
        # Initialize everything as OK
        self.clear_alert()
        self.monitor_thread.start()

        if self.test:
            return self.run_test()

        while not self.stopping:
            try:
                packet = self.get_packet(timeout=1)

                if not packet:
                    continue

                if self.conn:
                    self.conn.send((packet.raw, packet.rssi, packet.timestamp))

                # print(packet.serial, packet.seq, packet.orientation, packet.rssi, packet.vbatt)
                # pprint(packet.samples[-1])

                self.known_tags.add(packet.tag)

                if packet.chain < 50:  # ignore unless condition has lasted 5 cycles
                    continue

                if packet.orientation is not Orientation.UPRIGHT and packet.tag not in self.trouble_tags:
                    self.trouble_tags.add(packet.tag)
                    self.show_alert(packet.tag)
                elif packet.orientation is Orientation.UPRIGHT and packet.tag in self.trouble_tags:
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

        msg = "Tag %s is in distress!" % tag
        self.announce(msg)

        self.send_email("Bovine Intervention alert", msg)
        self.send_sms("Bovine Intervention alert: " + msg)

    def clear_alert(self):
        print("clearing alert")

        if self.announce_thread:
            self.announce_thread.stop()  # block

    def announce_cb(self, result):
        # reset to None
        self.announce_thread = None

    def announce(self, text: str):
        if self.announce_thread:
            self.announce_thread.join()  # block

        self.announce_thread = CommandThread(["./say.sh", text], self.announce_cb)

    def shutdown(self):
        self.stopping = True
        self.monitor_thread.join()

        if self.announce_thread:
            self.announce_thread.stop()

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
    parser.add_argument("-r", "--replay", metavar="FILE")

    args = parser.parse_args()

    if not args.replay:
        args.test = True
        print("Not replaying, entering test mode")

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
