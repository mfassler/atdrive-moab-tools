#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import os
import pickle
import struct
import numpy as np
import matplotlib.pyplot as plt

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
file_size = os.path.getsize(sys.argv[1])


nmea_vals = []
imu_vals = []
pid_vals = []
moab_vals = []
sbus_vals = []

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
        nmea_vals.append((timestamp, nmea_sentence))
    elif protocol == 'imu':
        imu.parse(raw_packet)
        # do something useful here
        imu_vals.append((timestamp, imu.shaft_a_pps, imu.shaft_b_pps))
    elif protocol == 'pid':
        pid.parse(raw_packet)
        # do something useful here
        pid_vals.append((timestamp, 
                          pid._sbus_steering, pid._sbus_throttle, pid.output, \
                          pid.K_p, pid._e, pid.K_i, pid._I, \
                          pid._target_speed, pid._actual_speed))
    elif protocol == 'moab':
        moab_vals.append((timestamp, raw_packet))
    elif protocol == 'sbus':
        sbus_vals.append((timestamp, raw_packet))
    else:
        print("Unknown protocol:", protocol)


last_good_position = f.tell()
while True:
    try:
        obj = pickle.load(f)
        #packets.append(obj)
        parse_obj(obj)
    except EOFError:  # normal, end-of-file behavior
        break
    except Exception as ee:  # possibly corrupted file
        print('failed to parse packet:', ee)
        break
    else:
        last_good_position = f.tell()

if last_good_position != file_size:
    print('GOOD: read %d bytes' % (last_good_position))
    print('BAD: failed to read %d bytes' % (file_size - last_good_position))


imu_data = np.array(imu_vals)
pid_data = np.array(pid_vals)



