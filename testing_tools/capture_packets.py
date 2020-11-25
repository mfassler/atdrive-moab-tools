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
nmea_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
nmea_sock.bind(("0.0.0.0", NMEA_RX_PORT))

IMU_RX_PORT = 27114
imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
imu_sock.bind(("0.0.0.0", IMU_RX_PORT))

PID_CONTROL_PORT = 27311
pid_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pid_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
pid_sock.bind(("0.0.0.0", PID_CONTROL_PORT))

MOAB_STATUS_PORT = 31337
moab_status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
moab_status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
moab_status_sock.bind(("0.0.0.0", MOAB_STATUS_PORT))

SBUS_RX_PORT = 31338
sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sbus_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sbus_sock.bind(("0.0.0.0", SBUS_RX_PORT))

BUTTON_PORT = 31345
button_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
button_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
button_sock.bind(("0.0.0.0", BUTTON_PORT))

#FOLLOW_AVOID_PORT = 52535
FOLLOW_AVOID_PORT = 52537
fa_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fa_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
fa_sock.bind(('0.0.0.0', FOLLOW_AVOID_PORT))


f = open(sys.argv[1], 'wb')

all_sockets = [nmea_sock, imu_sock, pid_sock, moab_status_sock, sbus_sock, button_sock, fa_sock]
while True:
    inputs, outputs, errors = select.select(all_sockets, [], [])
    for oneInput in inputs:
        if oneInput == nmea_sock:
            pkt, addr = nmea_sock.recvfrom(1500)
            pickle.dump(('nmea', time.time(), addr, pkt), f, -1)
            print('nmea', len(pkt))
        elif oneInput == imu_sock:
            pkt, addr = imu_sock.recvfrom(1500)
            pickle.dump(('imu', time.time(), addr, pkt), f, -1)
            print('imu', len(pkt))
        elif oneInput == pid_sock:
            pkt, addr = pid_sock.recvfrom(1500)
            pickle.dump(('pid', time.time(), addr, pkt), f, -1)
            print('pid', len(pkt))
        elif oneInput == moab_status_sock:
            pkt, addr = moab_status_sock.recvfrom(1500)
            pickle.dump(('moab', time.time(), addr, pkt), f, -1)
            print('moab', len(pkt))
        elif oneInput == sbus_sock:
            pkt, addr = sbus_sock.recvfrom(1500)
            pickle.dump(('sbus', time.time(), addr, pkt), f, -1)
            print('sbus', len(pkt))
        elif oneInput == button_sock:
            pkt, addr = button_sock.recvfrom(1500)
            pickle.dump(('button', time.time(), addr, pkt), f, -1)
            print('button', len(pkt))
        elif oneInput == fa_sock:
            pkt, addr = fa_sock.recvfrom(1500)
            pickle.dump(('folAvoid', time.time(), addr, pkt), f, -1)
            print('folAvoid', len(pkt))



