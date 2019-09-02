#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct


ODOMETRY_RX_PORT = 27112
odoSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
odoSock.bind(("0.0.0.0", ODOMETRY_RX_PORT))



# Speed is measured in pulses per second
current_speed = 0.0
avg_speed = 0.0


def calc_average_speed():
    global current_speed
    global avg_speed
    # TODO:  the running avg here should be a little more intelligent, maybe
    avg_speed = 0.95*avg_speed + 0.05*current_speed



def parseMe(pkt):
    global current_speed
    global avg_speed
    if len(pkt) != 4:
        print('wrong packet length')
    else:
        _nothing_, mtype, ms_since_last_event = struct.unpack('BBH', pkt)
        if mtype == 1 or mtype == 2:
            current_speed = 1000.0 / ms_since_last_event
            calc_average_speed()
        elif mtype == 0:
            max_speed = 1000.0 / ms_since_last_event
            if current_speed > max_speed:
                current_speed = max_speed
            if avg_speed > max_speed:
                avg_speed = max_speed
        else:
            print('wtf')


def get_all_packets(sock, bufsize=1500):
    sock.setblocking(0)
    while True:
        try:
            pkt, addr = sock.recvfrom(bufsize)
        except BlockingIOError as e:
            break
        else:
            parseMe(pkt)
    sock.setblocking(1)


while True:
    inputs, outputs, errors = select.select([odoSock], [], [])
    for oneInput in inputs:
        if oneInput == odoSock:
            get_all_packets(odoSock, 64)

    print("%.04f, %.04f" % (avg_speed, current_speed))



