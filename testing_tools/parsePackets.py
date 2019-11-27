#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import pickle
import struct

from ImuPacket import ImuPacket
imu = ImuPacket()


class PidPacket:
    def __init__(self):
        pass
    def parse(self, pkt):
        self._sbus_steering, self._sbus_throttle, self.output, \
        self.K_p, self._e, self.K_i, self._I, \
        self._target_speed, self._actual_speed = struct.unpack('HHfdddddd', pkt)

pid = PidPacket()

packets = []

f = open(sys.argv[1], 'rb')



def parse_obj(obj):
    assert len(obj) == 4
    protocol = obj[0]
    timestamp = obj[1]
    sender_addr = obj[2]
    raw_packet = obj[3]

    if protocol == 'nmea':
        # raw_packet is an ASCII NMEA sentence:
        nmea_sentence = raw_packet
        # do something useful here
    elif protocol == 'imu':
        imu.parse(raw_packet)
        # do something useful here
    elif protocol == 'pid':
        pid.parse(raw_packet)
        # do something useful here
    else:
        print("Unknown protocol:", protocol)



while True:
    try:
        obj = pickle.load(f)
        #packets.append(obj)
        parse_obj(obj)
    except EOFError:
        break




