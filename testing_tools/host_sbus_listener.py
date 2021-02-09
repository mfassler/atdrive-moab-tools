#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import select
import socket
import struct
import time


SBUS_RX_PORT = 31338
sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sbus_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sbus_sock.bind(("0.0.0.0", SBUS_RX_PORT))


t0 = time.time()

while True:
    pkt, addr = sbus_sock.recvfrom(64)
    t1 = time.time()
    fps = 1.0 / (t1-t0)
    t0 = t1
    if len(pkt) < 16:
        print("Error:  short packet.  len:", len(pkt))
    else:
        try:
            ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, \
            ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16, \
            failsafe, frame_lost = struct.unpack("HHHHHHHHHHHHHHHH??", pkt[:34])
        except Exception as e:
            print("failed to parse packet:", e)
        else:
            print(ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
                  ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16,
                  failsafe, frame_lost, fps)


