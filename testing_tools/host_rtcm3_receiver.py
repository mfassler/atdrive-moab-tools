#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import struct
import socket

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
udp_sock.bind(("0.0.0.0", 27117))


def CRC24(msg):
    CRC24_INIT = 0xb704ce
    CRC24_POLY = 0x1864cfb
    CRC24_OUTMASK = 0xffffff

    crc = 0 #CRC24_INIT
    for c in msg:

        crc ^= (c << 16)

        for i in range(8):
            crc <<= 1
            if (crc & 0x1000000):
                crc ^= 0x01864cfb

    return crc & CRC24_OUTMASK


packets = []



def check_packet(pkt):
    plen, = struct.unpack('!H', pkt[1:3])
    if len(pkt) != (plen + 6):
        print("WARN: wrong packet size.  Packet is %d bytes, but header says %d" % (len(pkt), plen+6))
        pkt = pkt[:plen+6]

    check = CRC24(pkt[0: -3])
    cksum = (pkt[-3] << 16) | (pkt[-2] << 8) | pkt[-1]

    if check == cksum:
        print('yay!')
    else:
        print('bad CRC.  Expected: 0x%x, got: 0x%x' % (check, cksum))



while True:
    pkt, addr = udp_sock.recvfrom(1500)
    print(len(pkt))
    packets.append(pkt)

    check_packet(pkt)



