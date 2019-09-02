
import time
import struct
import numpy as np


class ShaftEncoder:
    def __init__(self, encoding_distance):   #, sock):
        '''
        Parameters
        ----------
        encoding_distance : float, meters
            How far forward has the vehicle moved when the encoder makes a tick?
        '''
        self._enc_dist = encoding_distance
        #self.sock.bind(("0.0.0.0", udp_port_number))
        self.current_speed = 0.0
        self.avg_speed = 0.0
        self._us_since_last_event = 2**24 - 1

    def calc_average_speed(self):
        # TODO:  the running avg here should be a little more intelligent, maybe
        self.avg_speed = 0.95 * self.avg_speed + 0.05 * self.current_speed


    def parse_udp_packet(self, pkt):
        assert len(pkt) == 4
        mtype = pkt[3]

        us_since_last_event, = struct.unpack('I', pkt)
        self._us_since_last_event = 0x00ffffff & us_since_last_event

        if self._us_since_last_event < 1:
            self._us_since_last_event = 1

        if mtype == 1 or mtype == 2:
            if self._us_since_last_event >= 0xfffffe:
                self.current_speed = 0.0
            else:
                self.current_speed = 1000000.0 / self._us_since_last_event
            self.calc_average_speed()
        elif mtype == 0:
            if self._us_since_last_event >= 0xfffffe:
                max_speed = 0.0
            else:
                max_speed = 1000000.0 / self._us_since_last_event
            if self.current_speed > max_speed:
                self.current_speed = max_speed
            if self.avg_speed > max_speed:
                self.avg_speed = max_speed
        else:
            print('wtf')




    def get_all_packets(self, sock):
        sock.setblocking(0)
        while True:
            try:
                pkt, addr = sock.recvfrom(32)
            except BlockingIOError as e:
                break
            else:
                self.parse_udp_packet(pkt)
        sock.setblocking(1)


