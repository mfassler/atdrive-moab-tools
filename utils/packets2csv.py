#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import os
import argparse
import pickle
import pprint

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



class MyCsvWriter:
    def __init__(self, filename):
        self._f = open(filename, 'w')

    def write_header(self):
        txt = '%s, %s, %s, %s, %s\n' % (
            'ts',
            'latitude', 'longitude',
            'pid.target_speed', 'pid.actual_speed'
        )
        self._f.write(txt)

    def write_line(self, ts, imu, nmea, pid, folAvoid, r169):
        txt = '%s, %s, %s, %s, %s\n' % (
            ts,
            nmea.lat, nmea.lon,
            pid.target_speed, pid.actual_speed
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
            nmea.parse_packet(raw_packet)

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

    write_csv(packets, args.outfile, 0.0001)




