#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import socket
import numpy as np


SBUS_RX_PORT = 31338
sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sbus_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sbus_sock.bind(("0.0.0.0", SBUS_RX_PORT))


def convert_sbus_to_pwm(in_vals):
    SBUS_CENTER = 1024
    SBUS_TO_PWM_RATIO = 400.0 / 672.0
    PWM_CENTER = 1515

    out_vals_float = (in_vals.astype(float) - SBUS_CENTER) * SBUS_TO_PWM_RATIO + PWM_CENTER
    out_vals = np.round(out_vals_float).astype(np.uint16)

    return out_vals


while True:
    pkt, addr = sbus_sock.recvfrom(64)
    if len(pkt) < 32:
        print("Error:  short packet.  len:", len(pkt))
    else:
        in_vals = np.frombuffer(pkt[:32], np.uint16)
        print(in_vals)

        out_vals = convert_sbus_to_pwm(in_vals)
        sbus_sock.sendto(out_vals.tobytes(), ('localhost', 31538))


