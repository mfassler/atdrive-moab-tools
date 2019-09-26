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

        // 64 bits:
        int16_t compass_XYZ[3];  // external compass
        int16_t _padding1;  // the compiler seems to like 64-bit boundaries

        // 3 * 64 bits:
        char bnoData[22];  // internal IMU
        int16_t _padding2;  // the compiler seems to like 64-bit boundaries

        // 64 bits:
        float temperature; // degrees celsius, no need for high accuracy

        // Pressure:  typical sensor value is ~100000, with accuracy of +/- 12.0,
        // (don't forget to convert between Pa and hPa), so this is well
        // within the accuracy of float32
        float pressure;

        // 64 bits:
        uint16_t sbus_a;
        uint16_t sbus_b;
        uint16_t _padding3;
        uint16_t _padding4;

        // 64 bits:
        // TODO:  do we really need float64 for these numbers?
        double shaft_pps;

    } mData;


'''


class ImuPacket:
    def __init__(self):
        pass

    def parse(self, pkt):
        self.magX, self.magY, self.magZ, self._padding1, \
            self.qw, self.qx, self.qy, self.qz, \
            self.lax, self.lay, self.laz, self.gx, self.gy, self.gz, \
            self.imu_temp, self.calib_stat, \
            self._padding2, \
            self.temperature, self.pressure, \
            self.sbus_a, self.sbus_b, self._padding3, self._padding4, \
            self.shaft_pps = struct.unpack('<hhhhhhhhhhhhhhbBhffHHhhd', pkt)


imu = ImuPacket()

while True:
    pkt, addr = sock.recvfrom(128)
    imu.parse(pkt)

    #print(imu.shaft_pps)
    rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])

    hdg = -np.arctan2(rot[1, 0], rot[0,0])
    #print(rot)
    print("%x %.02f " % (imu.calib_stat, np.degrees(hdg)))
    #print("%.02f %.02f" % (imu.temperature, imu.pressure))

    #pktOut = rot.tobytes()
    #sock.sendto(pktOut, ('127.0.0.1', 12311))


