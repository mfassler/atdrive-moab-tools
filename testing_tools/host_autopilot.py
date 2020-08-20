#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct

MOAB_ADDRESS = ('192.168.35.231', 12346)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 0))


def set_values(steering, throttle):
    pkt = struct.pack('HH', steering, throttle)
    sock.sendto(pkt, MOAB_ADDRESS)


