#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct



MOAB_HOST = "192.168.11.231"
MOAB_PORT = 12346
IMU_CONFIG_PORT = 27115

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", IMU_CONFIG_PORT))


_config = None
def get_config():
    global _config
    sock.sendto(b'moabCRI', (MOAB_HOST, MOAB_PORT))
    pkt, addr = sock.recvfrom(256)

    if len(pkt) == 22:
        print('rx packet')
        _config = pkt
        parse_config(pkt)



def parse_config(pkt):
    acc_offset_x, acc_offset_y, acc_offset_z, \
    mag_offset_x, mag_offset_y, mag_offset_z, \
    gyr_offset_x, gyr_offset_y, gyr_offset_z, \
    acc_radius, mag_radius = struct.unpack('hhhhhhhhhhh', pkt)

    print("acc offset:", acc_offset_x, acc_offset_y, acc_offset_z)
    print("mag offset:", mag_offset_x, mag_offset_y, mag_offset_z)
    print("gyro offset:", gyr_offset_x, gyr_offset_y, gyr_offset_z)
    print("acc radius:", acc_radius)
    print("mag radius:", mag_radius)


_myConfig = b'\x00\x00\x00\x00\x00\x00\x92\x00F\x01\xdf\x01\xff\xff\x01\x00\x00\x00\xe8\x03f\x02'


def write_config(config):
    assert len(config) == 22
    pkt = b'moabCWI ' + config
    sock.sendto(pkt, (MOAB_HOST, MOAB_PORT))




