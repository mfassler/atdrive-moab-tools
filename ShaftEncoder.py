
import time
import struct
import numpy as np


class ShaftEncoder:
    def __init__(self, encoding_distance):
        '''
        Parameters
        ----------
        encoding_distance : float, meters
            How far forward has the vehicle moved when the encoder makes a tick?
        '''
        self._enc_dist = encoding_distance
        self._last_ts_system = time.time() # last timestamp using time.time()
        self._last_ts_moab_ms = None # last timestamp as told to us from the UDP packet (ms)
        self.speed = 0.0  # m/s, forward velocity estimate


    def parse_packet(self, pkt):
        # the packet is simply a uint64_t timestamp in ms
        odo_ts, = struct.unpack('Q', pkt)
        ts = time.time()
        if self._last_ts_moab_ms is None:
            pass
        else:
            tDelta_ms = odo_ts - self._last_ts_moab_ms
            self.speed = self._enc_dist / tDelta_ms * 1000.0
        self._last_ts_system = ts
        self._last_ts_moab_ms = odo_ts


    def update_speed_estimate(self):
        '''Must call this occasionally, even if odo packets don't arrive --
        If no packets arrive, then we know that the speed is slowing down
        '''
        tDelta = time.time() - self._last_ts_system
        max_possible_speed = self._enc_dist / tDelta
        if self.speed > max_possible_speed:
            self.speed = max_possible_speed


