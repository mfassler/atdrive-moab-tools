#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import struct
import time


COMPASS_RX_PORT = 27111
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", COMPASS_RX_PORT))


t0 = time.time()

while True:
    pkt, addr = sock.recvfrom(32)
    if len(pkt) != 6:
        print("udp packet is wrong length")
    else:
        x, y, z = struct.unpack("<hhh", pkt)
        print(x, y, z)


