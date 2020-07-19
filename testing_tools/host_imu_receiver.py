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

from ImuPacket import ImuPacket
imu = ImuPacket()


BNO055_RX_PORT = 27114
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind(("0.0.0.0", BNO055_RX_PORT))


t0 = time.time()



while True:
    pkt, addr = sock.recvfrom(128)
    imu.parse(pkt)

    #print(imu.shaft_pps)
    rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])

    hdg = -np.arctan2(rot[1, 0], rot[0,0])
    #print(rot)
    print("%x %.02f " % (imu.calib_stat, np.degrees(hdg)))
    #print("%.02f %.02f" % (imu.temperature, imu.pressure))

    print(imu.shaft_pps)
    #print(imu.shaft_a_pps, imu.shaft_b_pps)

    #pktOut = rot.tobytes()
    #sock.sendto(pktOut, ('127.0.0.1', 12311))


