#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import struct
import time
import pickle
import numpy as np
import transforms3d
import cv2 as cv
import matplotlib.pyplot as plt


IMU_RX_PORT = 27114
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", IMU_RX_PORT))

CMD_PORT = 23456
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.bind(("0.0.0.0", CMD_PORT))

t0 = time.time()


'''
this is the packet structure in C:

    struct multi_data {
        int16_t compass_XYZ[3];  // external compass
        int16_t _padding1;  // the compiler seems to like 64-bit boundaries
        char bnoData[20];  // internal IMU

        float temperature;
        float pressure;

        // TODO:  do we really need float64 for these numbers?
        double shaft_pps;

        int32_t _padding2;  // the compiler seems to like 64-bit boundaries
    } mData;
'''

vals = []


amap = np.ones((800, 800, 3), np.uint8) * 255
blankLine = np.ones((800, 1, 3), np.uint8) * 255
# Draw the blue gridlines:
for y in np.arange(100, 800, 100):
    blankLine[y-1, 0] = (255, 128, 128)
    blankLine[y, 0] = (180, 0, 0)
    blankLine[y+1, 0] = (255, 128, 128)

cv.imshow('asd', amap)
cv.moveWindow('asd', 20, 20)
cv.waitKey(1)


def update_scrolling_data(actual_pps, target_pps):
    global amap
    global blankLine

    amap = np.hstack( (amap[:, 1:800, :], blankLine))

    py = int(round(800 - actual_pps * 10))
    if py < 0 :
        py = 0
    elif py > 798:
        py = 798

    tpy = int(round(800 - target_pps * 10))
    if tpy < 0 :
        tpy = 0
    elif tpy > 798:
        tpy = 798

    #amap[py, 799] = (0,0,220)
    cv.circle(amap, (798, tpy), 3, (0,96, 0), -1)
    cv.circle(amap, (798, py), 2, (0,0,220), -1)

    cv.imshow('asd', amap)
    cv.waitKey(1)




class SpeedControl:
    def __init__(self):
        self.moab_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.moab_sock.bind(("0.0.0.0", 0))
        self._last_sbus_time = 0
        self._MOAB_COMPUTER = "192.168.29.201"
        self._MOAB_PORT = 12346
        self._ts = [time.time()] * 45
        self._error = [0.0] * 45
        self._actual_pps = 0
        self._target_pps = 0
        self.K_p = 10.0
        self.K_i = 5.0
        self.K_d = 0.0
        self.wtf__I = 0
        self.wtf_sbus = -1
        self._offset = 0.0

    def set_actual(self, actual_pps):
        self._actual_pps = actual_pps

    def set_target(self, target_pps):
        self._target_pps = target_pps

    def update_error(self):
        ts = time.time()
        error = self._target_pps - self._actual_pps
        self._ts.append(ts)
        self._error.append(error)
        while len(self._ts) > 15:
            self._ts.pop(0)
        while len(self._error) > 15:
            self._error.pop(0)

    def maybe_do_something(self):
        ts = time.time()
        if (ts - self._last_sbus_time) > 0.1:
            self._last_sbus_time = ts
            self.update_error()

            I = 0
            for i in range(len(self._error) - 1):
                tDelta = self._ts[i+1] - self._ts[i]
                e = self._error[i+1] # - self._error[i]
                I += e*tDelta

            if I > 20:
                I = 20
            elif I < -20:
                I = -20

            self.wtf__I = I

            #e = self._error[-1]

            self._offset += 0.5 * I

            #print("%.02f %.02f" % (e, self._offset))

            #output = self.K_p * self._error[-1] + self.K_i * I # + self._offset
            output = self.K_p * self._error[-1] + self._offset
            if output < 0:
                output = 0
            elif output > 370:
                output = 370

            sbus_val = 1024 + int(round(output))

            self.wtf_sbus = sbus_val

            udpPacket = struct.pack('HH', 1024, sbus_val)
            self.moab_sock.sendto(udpPacket, (self._MOAB_COMPUTER, self._MOAB_PORT))



speedControl = SpeedControl()


target_pps = 0

while True:
    inputs, outpus, errors = select.select([sock, cmd_sock], [], [])
    for oneInput in inputs:
        if oneInput == sock:
            pkt, addr = sock.recvfrom(128)
            if len(pkt) != 48:
                print("udp packet is wrong length:", len(pkt))
            else:
                magX, magY, magZ, _nothing1, \
                qw, qx, qy, qz, lax, lay, laz, gx, gy, gz, \
                temperature, pressure, sbus_a, sbus_b, shaft_pps = \
                struct.unpack("<hhhhhhhhhhhhhhffHHd", pkt)

                #print(temperature, pressure, shaft_pps, target_pps)

                update_scrolling_data(shaft_pps, target_pps)
                speedControl.set_actual(shaft_pps)
                speedControl.maybe_do_something()

        if oneInput == cmd_sock:
            pkt, addr = cmd_sock.recvfrom(256)
            try:
                target_pps = float(pkt)
                if target_pps < 0:
                    target_pps = 0
                elif target_pps > 55:
                    target_pps = 55
            except:
                print(' *********** wtf *********')
            else:
                speedControl.set_target(target_pps)



