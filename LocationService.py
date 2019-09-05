#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct
import numpy as np

from UbloxParser import UbloxParser
from NmeaParser import NmeaParser
from ShaftEncoder import ShaftEncoder

from utils import get_new_gps_coords
from MavlinkHandler import MavlinkHandler

import ROBOT_CONFIG as config

mavlink = MavlinkHandler(config.MAVLINK_IP_ADDRESS)

#ublox = UbloxParser(mavlink)
nmea = NmeaParser(mavlink)

shaft = ShaftEncoder(config.SHAFT_ENCODER_DISTANCE)


## We need to listen for certain UDP packets coming from the Moab:
#UBLOX_RX_PORT = 27110
#gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#gps_sock.bind(("0.0.0.0", UBLOX_RX_PORT))

COMPASS_RX_PORT = 27111
mag_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mag_sock.bind(("0.0.0.0", COMPASS_RX_PORT))

ODOMETRY_RX_PORT = 27112
odo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
odo_sock.bind(("0.0.0.0", ODOMETRY_RX_PORT))

NMEA_RX_PORT = 27113
gps_sock_nmea = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gps_sock_nmea.bind(("0.0.0.0", NMEA_RX_PORT))

LIDAR_NAV_RX_PORT = 11546
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.bind(("0.0.0.0", LIDAR_NAV_RX_PORT))

## We will send lat, lon, heading, and speed to the Autopilot program
NAV_PORT_OUT = 27201
nav_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
nav_sock.bind(("127.0.0.1", 0))

xx_avg = 0.0
yy_avg = 0.0
zz_avg = 0.0
rot_matrix = np.array([[1, 0], [0, 1]])
est_heading = 0.0

compass_rot_offset = np.radians(config.compass_rotation_offset_degrees)


def parse_compass_packet(pkt):
    assert len(pkt) == 6
    global xx_avg
    global yy_avg
    global zz_avg
    global rot_matrix
    global est_heading

    x, y, z = struct.unpack("<hhh", pkt)

    xx = (x - config.compass_x_center) / config.compass_x_range
    yy = (y - config.compass_x_center) / config.compass_y_range
    zz = (z - config.compass_x_center) / config.compass_z_range

    xx_avg = 0.5 * xx_avg + 0.5 * xx
    yy_avg = 0.5 * yy_avg + 0.5 * yy
    zz_avg = 0.5 * zz_avg + 0.5 * zz

    # rotation matrix:
    avg_rot = np.arctan2(xx_avg, yy_avg) + compass_rot_offset
    est_heading = np.degrees(avg_rot)
    cosYaw = np.cos(avg_rot)
    sinYaw = np.sin(avg_rot)
    rot_matrix = np.array([[ cosYaw, sinYaw],
                           [-sinYaw, cosYaw]])

    norm = np.sqrt(xx**2 + yy**2 + zz**2)

    #hdg = np.degrees(np.arctan2(xx, yy)) - 7.5
    mavlink.send_attitude(0, 0, est_heading)
    #print('%.0f ' % (hdg), norm)


est_lat = None
est_lon = None

ref_angle = np.radians(config.lidar_ref_angle)
rot_matrix = np.array([[ np.cos(ref_angle), np.sin(ref_angle)],
                       [-np.sin(ref_angle), np.cos(ref_angle)]])

def parse_lidar_nav_packet(udpPacket):
    global config
    global rot_matrix
    if len(udpPacket) != 72:
        print('wrong packet length from lidar_nav')
    else:
        xfrm = np.frombuffer(pkt).reshape((3,3))
        heading = np.arctan2(xfrm[1,0], xfrm[1,1])
        #amap = np.copy(avoidance_areas)

        startPos = np.dot(xfrm[:2, :2].T, xfrm[:2, 2])
        x_real = startPos[1]
        y_real = -startPos[0]

        newPos = np.dot(rot_matrix, [x_real, y_real])

        newLat, newLon = get_new_gps_coords(
                config.lidar_ref_lat, config.lidar_ref_lon, newPos[1], newPos[0])

        return newLat, newLon, heading+ref_angle



def get_last_packet(sock, pkt_len=1500):
    '''Empty out the UDP recv buffer and return only the final packet
    (in case the GUI is slower than the data flow)
    '''
    sock.setblocking(0)
    data = None
    addr = None
    cont=True
    while cont:
        try:
            tmpData, addr = sock.recvfrom(pkt_len)
        except Exception as ee:
            #print(ee)
            cont=False
        else:
            if tmpData:
                if data is not None:
                    #print('throwing away a packet (GUI is too slow)')
                    pass
                data = tmpData
            else:
                cont=False
    sock.setblocking(1)
    return data, addr


_last_odo_ts = None
while True:
    inputs, outputs, errors = select.select([gps_sock_nmea, mag_sock, odo_sock, lidar_sock, mavlink._sock], [], [], 0.2)
    for oneInput in inputs:
        if False:
            # TODO:  what is the max packet size of Ublox?
            pkt, addr = gps_sock.recvfrom(1500)
            try:
                ublox.parse_ublox_packet(pkt)
            except Exception as ee:
                print('failed to parse Ublox packet:', ee)
            else:
                if ublox.gpsFix == 3:
                    if est_lat is None:
                        est_lat = ublox.lat
                    if est_lon is None:
                        est_lon = ublox.lon
                    est_lat = 0.9 * est_lat + 0.1 * ublox.lat
                    est_lon = 0.9 * est_lon + 0.1 * ublox.lon
                mavlink.send_gps(est_lat, est_lon, ublox.alt)
                #mavlink.send_raw_gps(ts, gpsFix, lat, lon, alt, numSV)

        elif oneInput == gps_sock_nmea:
            pkt, addr = gps_sock_nmea.recvfrom(1500)
            try:
                nmea.parse_nmea_packet(pkt)
            except Exception as ee:
                print('failed to parse nmea packet:', ee)
            else:
                if nmea.gpsFix == 3:
                    est_lat = nmea.lat
                    est_lon = nmea.lon
                    mavlink.send_gps(est_lat, est_lon, nmea.alt)

        elif oneInput == mag_sock:
            pkt, addr = mag_sock.recvfrom(1500)
            try:
                parse_compass_packet(pkt)
            except Exception as ee:
                print('failed to parse compass packet:', ee)

        elif oneInput == odo_sock:
            shaft.get_all_packets(odo_sock)

        elif oneInput == mavlink._sock:
            pkt, addr = mavlink._sock.recvfrom(512)
            #print("MAVLINK, rx %d bytes" % (len(pkt)))
            try:
                msgs = mavlink._mav.parse_buffer(pkt)
            except Exception as ee:
                print("mavlink._mav.parse_buffer() failed:", ee)
            else:
                for oneMsg in msgs:
                    mavlink.do_message(oneMsg)

        elif oneInput == lidar_sock:
            pkt, addr = get_last_packet(lidar_sock, 512)
            try:
                lat, lon, hdg = parse_lidar_nav_packet(pkt)
            except Exception as ee:
                print("parse_lidar_nav_packet() failed:", ee)
            else:
                est_heading = np.degrees(hdg)
                if est_lat is None:
                    est_lat = lat
                if est_lon is None:
                    est_lon = lon
                est_lat = 0.9*est_lat + 0.1*lat
                est_lon = 0.9*est_lon + 0.1*lon
                mavlink.send_attitude(0, 0, est_heading)
                mavlink.send_gps(est_lat, est_lon, 4000)


    #shaft.update_speed_estimate()
    print('speed: %.02f' % (shaft.current_speed))
    mavlink.send_vfr_hud(shaft.current_speed)
    if est_lat is not None and est_lon is not None and est_heading is not None:
        nav_udp_packet = struct.pack('!dddd', est_lat, est_lon, est_heading, shaft.current_speed)
        nav_sock.sendto(nav_udp_packet, ('127.0.0.1',NAV_PORT_OUT))



