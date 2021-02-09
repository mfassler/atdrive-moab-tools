#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import pickle
import struct
import socket

from ImuPacket import ImuPacket
imu = ImuPacket()


# We will re-transmit the packets to this location:
RX_HOST = '127.0.0.1'
NMEA_RX_PORT = 27113
IMU_RX_PORT = 27114


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 0))


packets = []

f = open(sys.argv[1], 'rb')

while True:
    try:
        obj = pickle.load(f)
        packets.append(obj)
    except EOFError:
        break

f.close()

START_POS = 0

ts0 = time.time()
for i in range(START_POS, len(packets) - 1):

    protocol = packets[i][0]
    packet = packets[i][3]
    # Transmit the packet to LocationServices:
    if protocol == 'nmea':
        s.sendto(packet, (RX_HOST, NMEA_RX_PORT))
    elif protocol == 'imu':
        s.sendto(packet, (RX_HOST, IMU_RX_PORT))
        imu.parse(packet)
        print(imu.sbus_a, imu.sbus_b)
    else:
        print(" ******* Unknown protocol:", protocol)


    # Simulated Time Stamps:
    sts0 = packets[i][1]
    sts1 = packets[i+1][1]
    # Simulated Time Delay:
    stDelay = sts1 - sts0

    # Actual Time has passed:
    ts1 = time.time()
    tDelay = ts1 - ts0
    ts0 = ts1

    realDelay = stDelay - tDelay #+ 0.2
    #print(realDelay)
    if realDelay > 0.0:
        time.sleep(realDelay)


