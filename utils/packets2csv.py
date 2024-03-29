#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import os
import argparse
import pickle
import pprint
import numpy as np
import transforms3d

import protocols
from protocols.SbusParser import Flight_Mode
from protocols.ImuPacket import Moab_mode, Rc_Controller_Source



def load_packets_file(filename):
    packets = []
    pktTypes = {}

    file_size = os.path.getsize(sys.argv[1])
    f = open(filename, 'rb')

    last_good_position = f.tell()
    while True:
        try:
            obj = pickle.load(f)
            packets.append(obj)
            #parse_obj(obj)
        except EOFError:  # normal, end-of-file behavior
            break
        except Exception as ee:  # possibly corrupted file
            print('failed to parse packet:', ee)
            break
        else:
            last_good_position = f.tell()

    if last_good_position != file_size:
        print('GOOD: read %d bytes (%d packets)' % (last_good_position, len(packets)))
        print('BAD: failed to read %d bytes' % (file_size - last_good_position))

    f.close()

    # Make sure all the timestamps are in order:
    ts = 0
    for protocol, timestamp, sender_addr, raw_packet in packets:
        assert timestamp > ts, 'WARN: Timestamps in pkl file are not in order'
        ts = timestamp
        if protocol not in pktTypes:
            pktTypes[protocol] = 1
        else:
            pktTypes[protocol] += 1

    print()
    print('File contains these packet types:')
    pprint.pprint(pktTypes)
    print()

    return packets



# Moab is mounted with "forward" pointing "left"
#   (ie:  ethernet cable is pointing "right")
IMU_XFRM = np.array([
    [ 0, 1, 0],
    [-1, 0, 0],
    [ 0, 0, 1]
])

SHAFT_ENCODER_DISTANCE = 0.155 # 15.5 cm per tick


class MyCsvWriter:
    def __init__(self, filename):
        self._f = open(filename, 'w')

        self._field_names = (
            'ts',
            'imu.lax', 'imu.lay', 'imu.laz', 'imu.gx', 'imu.gy', 'imu.gz',
            'imu.temperature', 'imu.pressure', 'imu.sbus_a', 'imu.sbus_b', 'imu.moab_mode',
            'imu.adc0',
            'pitch', 'roll', 'yaw',
            'shaft_a_est_speed', 'shaft_b_est_speed',

            'latitude', 'longitude', 'gps.speed', 'gps.true_course', 'gps.GGA_fix',

            'pid.sbus_steering', 'pid.sbus_throttle', 'pid.output', 'pid.K_p',
            'pid.e', 'pid.K_i', 'pid.I', 'pid.target_speed', 'pid.actual_speed',

            'folAvoid.follow_r', 'folAvoid.follow_angle', 'folAvoid.avoid_color', 'folAvoid.avoid_x', 'folAvoid.avoid_y',

            'r169.btn_start', 'r169.btn_back', 'r169.btn_logi', 'r169.btn_wtf',
            'r169.btn_LB', 'r169.btn_LT', 'r169.btn_RB', 'r169.btn_RT',
            'r169.btn_Y', 'r169.btn_A', 'r169.btn_B', 'r169.btn_X',
            'r169.btn_up', 'r169.btn_down', 'r169.btn_right', 'r169.btn_left',
            'r169.leftjoy_lr', 'r169.leftjoy_ud', 'r169.rightjoy_lr', 'r169.rightjoy_ud'

        )

        numFields = len(self._field_names)
        self._PRINTF_STRING = '%s, ' * (numFields - 1) + '%s\n'

    def write_header(self):
        txt = self._PRINTF_STRING % self._field_names
        self._f.write(txt)


    def write_line(self, ts, imu, nmea, pid, folAvoid, r169):
        rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])

        rot = np.dot(rot, IMU_XFRM)
        pitch, roll, _yaw = transforms3d.euler.mat2euler(rot)
        yaw = -_yaw

        txt = self._PRINTF_STRING % (
            ts,
            imu.lax, imu.lay, imu.laz, imu.gx, imu.gy, imu.gz,
            imu.temperature, imu.pressure, imu.sbus_a, imu.sbus_b, imu.moab_mode,
            imu.adc0,
            np.degrees(pitch), np.degrees(roll), np.degrees(yaw),
            imu.shaft_a_pps * SHAFT_ENCODER_DISTANCE, imu.shaft_b_pps * SHAFT_ENCODER_DISTANCE,

            nmea.lat, nmea.lon, nmea.speed, nmea.true_course, nmea.GGA_fix,

            pid.sbus_steering, pid.sbus_throttle, pid.output, pid.K_p,
            pid.e, pid.K_i, pid.I, pid.target_speed, pid.actual_speed,

            folAvoid.follow_r, folAvoid.follow_angle, folAvoid.avoid_color, folAvoid.avoid_x, folAvoid.avoid_y,

            r169.btn_start, r169.btn_back, r169.btn_logi, r169.btn_wtf,
            r169.btn_LB, r169.btn_LT, r169.btn_RB, r169.btn_RT,
            r169.btn_Y, r169.btn_A, r169.btn_B, r169.btn_X,
            r169.btn_up, r169.btn_down, r169.btn_right, r169.btn_left,
            r169.leftjoy_lr, r169.leftjoy_ud, r169.rightjoy_lr, r169.rightjoy_ud
        )
        self._f.write(txt)

    def close(self):
        self._f.close()



def write_csv(packets, outfile, interval):
    csv = MyCsvWriter(outfile)
    csv.write_header()

    ppkts = {}
    imu = protocols.ImuPacket()
    nmea = protocols.NmeaParser()
    pid = protocols.PidParser()
    sbus = protocols.SbusParser()
    r169 = protocols.Radio169()
    folAvoid = protocols.FollowAvoid()

    # Get the first timestamp:
    _nothing1, t0, _nothing2, _nothing3 = packets[0]

    for protocol, timestamp, sender_addr, raw_packet in packets:
        if protocol == 'imu':
            print('imu')
            imu.parse(raw_packet)

        elif protocol == 'nmea':
            print('    nmea')
            try:
                nmea.parse_packet(raw_packet)
            except Exception as e:
                print('BAD NMEA packet:', e, raw_packet)

        elif protocol == 'pid':
            print('         pid')
            pid.parse_packet(raw_packet)

        elif protocol == 'folAvoid':
            print('             folAvoid')
            folAvoid.parse_packet(raw_packet)

        elif protocol == 'sbus':
            print('                      sbus')
            flight_mode = sbus.parse_packet(raw_packet)

        elif protocol == 'r169':
            print('                           r169')
            r169.parse_payload(raw_packet)

        else:
            print('   ********************* UNKOWN PROTOCOL:', protocol)

        if protocol not in ppkts:
            ppkts[protocol] = [[timestamp, sender_addr, raw_packet]]
        else:
            ppkts[protocol].append([timestamp, sender_addr, raw_packet])

        while timestamp > (t0 + interval):
            t0 += interval
            csv.write_line(t0, imu, nmea, pid, folAvoid, r169)

    csv.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert a packets file into CSV, with regular time intervals')
    parser.add_argument('infile', metavar='infile.pkl', help='input file (Python pickle format)')
    parser.add_argument('outfile', metavar='outfile.csv', help='output file (CSV)')

    args = parser.parse_args()

    packets = load_packets_file(args.infile)

    write_csv(packets, args.outfile, 0.1)




