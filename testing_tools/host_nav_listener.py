#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import socket
import struct

NAV_PORT = 27201

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind(('0.0.0.0', NAV_PORT))

while True:
    pkt, addr = sock.recvfrom(64)
    try:
        est_lat, est_lon, est_heading, spd, = struct.unpack('!dddd', pkt)
        print(est_lat, est_lon, est_heading, spd)
    except Exception as e:
        print('failed to parse packet:', e)


