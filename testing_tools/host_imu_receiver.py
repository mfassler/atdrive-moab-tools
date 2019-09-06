#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import struct
import time
import numpy as np
import transforms3d



BNO055_RX_PORT = 27114
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", BNO055_RX_PORT))


t0 = time.time()

while True:
    pkt, addr = sock.recvfrom(64)
    if len(pkt) != 20:
        print("udp packet is wrong length")
    else:
        qw, qx, qy, qz, lax, lay, laz, gx, gy, gz = struct.unpack("<hhhhhhhhhh", pkt)
        print(lax, lay, laz, gx, gy, gz)

        rot = transforms3d.quaternions.quat2mat([qw, qx, qy, qz])

        hdg = -np.arctan2(rot[1, 0], rot[0,0])
        #print(rot)
        print(np.degrees(hdg))
