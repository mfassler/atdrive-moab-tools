#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import time
import pickle


NMEA_RX_PORT = 27113
nmea_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
nmea_sock.bind(("0.0.0.0", NMEA_RX_PORT))

BNO055_RX_PORT = 27114
imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock.bind(("0.0.0.0", BNO055_RX_PORT))

PID_CONTROL_PORT = 27311
pid_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pid_sock.bind(("0.0.0.0", PID_CONTROL_PORT))

f = open(sys.argv[1], 'wb')

while True:
    inputs, outputs, errors = select.select([nmea_sock, imu_sock, pid_sock], [], [])
    for oneInput in inputs:
        if oneInput == nmea_sock:
            data, addr = nmea_sock.recvfrom(1500)
            pickle.dump(('nmea', time.time(), addr, data), f, -1)
            print('nmea', len(data))
        elif oneInput == imu_sock:
            data, addr = imu_sock.recvfrom(1500)
            pickle.dump(('imu', time.time(), addr, data), f, -1)
            print('imu', len(data))
        elif oneInput == pid_sock:
            data, addr = pid_sock.recvfrom(1500)
            pickle.dump(('pid', time.time(), addr, data), f, -1)
            print('pid', len(data))

