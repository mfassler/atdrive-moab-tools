#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct


BUTTON_PORT = 31345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', BUTTON_PORT))



while True:
    data, addr = sock.recvfrom(64)
    if len(data) != 8:
        print("Wrong data length")
    else:
        button_ts_ms, = struct.unpack('Q', data)
        print("Button:", button_ts_ms)




