#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct
import numpy as np

from NmeaParser import NmeaParser

nmea = NmeaParser()


NMEA_RX_PORT = 27113
gps_sock_nmea = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gps_sock_nmea.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
gps_sock_nmea.bind(("0.0.0.0", NMEA_RX_PORT))


while True:
    pkt, addr = gps_sock_nmea.recvfrom(1500)
    try:
        nmea.parse_nmea_packet(pkt)
    except Exception as ee:
        print('failed to parse nmea packet:', ee)
    else:
        print(time.time())
        print(nmea.ts, nmea.lat, nmea.lon)

