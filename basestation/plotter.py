from collections import deque
import itertools
from multiprocessing.connection import Listener
import threading
from time import time, sleep

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3

from structures import Orientation, RadioPacket, IMUSample

ADDRESS = ("0.0.0.0", 56700)
WINDOW = 10


class BackgroundThread(threading.Thread):
    def __init__(self, callback, *args, start=True, **kwargs):
        threading.Thread.__init__(self)
        assert callable(callback)
        self.callback = callback
        self.args = args
        self.kwargs = kwargs
        self.daemon = True

        if start:
            self.start()

    def run(self):
        self.callback(*self.args, **self.kwargs)


class Plotter:
    def __init__(self, maxlen=128):
        self.queue = deque(maxlen=maxlen)
        self.ani = None
        self.window = deque()

        self.fig = plt.figure()

        ax1 = self.ax1 = self.fig.add_subplot(231)
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('acceleration (m/s^2)')

        axl = self.ax_line = Line2D([], [], color='red')
        ayl = self.ay_line = Line2D([], [], color='green')
        azl = self.az_line = Line2D([], [], color='blue')
        ax1.add_line(axl)
        ax1.add_line(ayl)
        ax1.add_line(azl)
        plt.legend((axl, ayl, azl), ('Ax', 'Ay', 'Az'))

        ax4 = self.ax4 = self.fig.add_subplot(232)
        ax4.set_xlabel('time (s)')
        ax4.set_ylabel('angular velocity (deg/s)')
        wxl = self.wx_line = Line2D([], [], color='red')
        wyl = self.wy_line = Line2D([], [], color='green')
        ax4.add_line(wxl)
        ax4.add_line(wyl)
        ax4.set_ylim([-360, 360])
        ax4.set_xlim([0, WINDOW])
        plt.legend((wxl, wyl), ('ωx', 'ωy'))

        self.ax1.set_ylim([-10, 10])
        self.ax1.set_xlim([0, WINDOW])

        ax2 = self.ax2 = self.fig.add_subplot(235)
        dl = self.disp_line = Line2D([], [], color='purple')
        dl.set_label('Displacement (m)')
        ax2.add_line(dl)
        ax2.legend()
        ax2.set_ylim([0, 2])
        ax2.set_xlim([0, WINDOW])

        ax3 = self.ax3 = self.fig.add_subplot(233)
        ol = self.ol = Line2D([], [], color='black')
        ola = self.ola = Line2D([], [], color='red', linewidth=2)
        ole = self.ole = Line2D([], [], color='red', marker='o', markeredgecolor='r')
        ax3.set_ylim([-180, 180])
        ax3.set_xlim([-90, 90])
        ax3.add_line(ol)
        ax3.add_line(ola)
        ax3.add_line(ole)
        ax3.set_xlabel('roll')
        ax3.set_ylabel('pitch')
        self.o_ann = None

        ax5 = self.ax5 = self.fig.add_subplot(234)
        ax5.set_ylim([0, 5])
        ax5.set_xlim([0, WINDOW])
        vl = self.v_line = Line2D([], [], color='orange')
        vl.set_label("Battery voltage (V)")
        ax5.add_line(vl)
        ax5.legend()

        ax6 = self.ax6 = self.fig.add_subplot(236)
        #ax.set_yscale('log')
        ax6.set_ylim([-115, 0])
        ax6.set_xlim([0, WINDOW])
        sl = self.rssi_line = Line2D([], [], color='blue')
        sl.set_label("RSSI (dB)")
        ax6.add_line(sl)
        ax6.legend()

        self.listener_thread = None

    def listener(self):
        with Listener(ADDRESS) as listener:
            while True:
                try:
                    with listener.accept() as conn:
                        print('connection accepted from', listener.last_accepted)
                        while True:
                            data = conn.recv()
                            packet = RadioPacket.from_bytes(*data)
                            self.queue.append(packet)
                except Exception:
                    continue


    def run(self):
        self.listener_thread = BackgroundThread(self.listener)
        self.ani = animation.FuncAnimation(self.fig, self._update, blit=False, interval=2, repeat=False)
        plt.show()

    def _update(self, frame):
        self.window.extend(self.queue)
        self.queue.clear()

        if not self.window:
            return

        cutoff = time() - WINDOW

        while self.window[0].timestamp < cutoff:
            p = self.window.popleft()

            if not self.window:
                break
            elif self.window[0].timestamp >= cutoff:
                # we want to keep the one packet over
                self.window.appendleft(p)
                break

        if not self.window:
            return

        samples = list(itertools.chain(*(p.samples for p in self.window)))
        p_times = list(itertools.accumulate(sum(s.dt for s in p.samples) for p in self.window))
        s_times = list(itertools.accumulate(s.dt for s in samples))

        self.ax_line.set_data(s_times, [s.ax for s in samples])
        self.ay_line.set_data(s_times, [s.ay for s in samples])
        self.az_line.set_data(s_times, [s.az for s in samples])

        self.wx_line.set_data(s_times, [s.wx for s in samples])
        self.wy_line.set_data(s_times, [s.wy for s in samples])

        self.disp_line.set_data(p_times, [p.displacement for p in self.window])
        self.v_line.set_data(p_times, [p.vbatt for p in self.window])

        pitches = [p.angle_y for p in self.window]
        pitches = [(180 + p) if p < 0 else (-180 + p) for p in pitches]
        rolls = [p.angle_x for p in self.window]
        self.ol.set_data(rolls, pitches)
        self.ola.set_data([rolls[-10:]], [pitches[-10:]])
        self.ole.set_data([rolls[-1]], [pitches[-1]])

        if self.o_ann:
            self.o_ann.remove()

        orientation = self.window[-1].orientation
        cycles = self.window[-1].chain

        if cycles > 10:
            ann = orientation.name.lower()
        else:
            ann = "steady" + "." * (cycles//3)

        self.o_ann = self.ax3.annotate(ann, xy=(rolls[-1], pitches[-1]),
                                       xytext=(rolls[-1] + 5, pitches[-1] + 5),
                                       color = "purple", fontsize=12)

        self.rssi_line.set_data(p_times, [p.rssi for p in self.window])

        #self.ax1.set_xticklabels(times)


if __name__ == "__main__":
    p = Plotter(None)
    p.run()
