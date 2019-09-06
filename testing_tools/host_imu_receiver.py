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


'''
this is the packet structure in C:

    struct multi_data {
        int16_t compass_XYZ[3];  // external compass
        int16_t _padding1;  // the compiler seems to like 64-bit boundaries
        char bnoData[20];  // internal IMU
        int32_t _padding2;  // the compiler seems to like 64-bit boundaries

        // TODO:  do we really need float64 for these numbers?
        double shaft_pps;
        double temperature;
        double pressure;
    } mData;

'''


while True:
    pkt, addr = sock.recvfrom(128)
    if len(pkt) != 56:
        print("udp packet is wrong length:", len(pkt))
    else:
        magX, magY, magZ, _nothing1, qw, qx, qy, qz, lax, lay, laz, gx, gy, gz, _nothing2, shaft_pps, temperature, pressure = struct.unpack("<hhhhhhhhhhhhhhlddd", pkt)
        print(magX, magY, magZ, lax, lay, laz, gx, gy, gz, shaft_pps, temperature, pressure)

        rot = transforms3d.quaternions.quat2mat([qw, qx, qy, qz])

        hdg = -np.arctan2(rot[1, 0], rot[0,0])
        #print(rot)
        #print(np.degrees(hdg))

        pktOut = rot.tobytes()
        sock.sendto(pktOut, ('127.0.0.1', 12311))
