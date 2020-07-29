#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import select
import socket
import struct
import numpy as np
import transforms3d


from UbloxParser import UbloxParser
from NmeaParser import NmeaParser
from ShaftEncoder import ShaftEncoder
from CalcHeading import CalcHeading
from SbusParser import SbusParser, Flight_Mode

from ImuPacket import ImuPacket

sbus = SbusParser()
imu = ImuPacket()
calcHeading = CalcHeading()

from utils import get_new_gps_coords
from misc_utils import get_last_packet
from MavlinkHandler import MavlinkHandler

try:
    import ROBOT_CONFIG as config
except:
    import ROBOT_CONFIG_default as config


IMU_XFRM = None
if hasattr(config, 'IMU_XFRM'):
    # The Moab can be mounted in any orientation, but the
    # user will have to provide the 3x3 rotation matrix:
    IMU_XFRM = np.array(config.IMU_XFRM)


mavlink = MavlinkHandler(config.MAVLINK_IP_ADDRESS)

#ublox = UbloxParser(mavlink)
nmea = NmeaParser()


## We need to listen for certain UDP packets coming from the Moab:
#UBLOX_RX_PORT = 27110
#gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#gps_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
#gps_sock.bind(("0.0.0.0", UBLOX_RX_PORT))

IMU_RX_PORT = 27114
imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
imu_sock.bind(("0.0.0.0", IMU_RX_PORT))

NMEA_RX_PORT = 27113
gps_sock_nmea = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gps_sock_nmea.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
gps_sock_nmea.bind(("0.0.0.0", NMEA_RX_PORT))

LIDAR_NAV_RX_PORT = 11546
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.bind(("0.0.0.0", LIDAR_NAV_RX_PORT))

SBUS_PORT = 31338
sbus_sock = None
#sbus_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sbus_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
#sbus_sock.bind(("0.0.0.0", SBUS_PORT))

GCS_MESSAGES_PORT = 31339
gcs_msgs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gcs_msgs_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
gcs_msgs_sock.bind(("127.0.0.1", GCS_MESSAGES_PORT))

## We will send lat, lon, heading, and speed to the Autopilot program
nav_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
nav_sock.bind(("0.0.0.0", 0))


est_speed = 0.0
est_lat = None
est_lon = None
est_heading = None


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


#allSockets = [gps_sock_nmea, imu_sock, lidar_sock, mavlink._sock, sbus_sock]
allSockets = [gps_sock_nmea, imu_sock, lidar_sock, mavlink._sock, gcs_msgs_sock]
while True:
    inputs, outputs, errors = select.select(allSockets, [], [], 0.2)
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
                nmea.parse_packet(pkt)
            except Exception as ee:
                print('failed to parse nmea packet:', ee)
            else:
                if nmea.RMC_status:
                    est_lat = nmea.lat
                    est_lon = nmea.lon
                    if nmea.true_course is not None:
                        # only update if we are driving straight forward:
                        if np.abs(imu.sbus_a - 1024) < 20 and imu.sbus_b > 1030:
                            calcHeading.calibrate_from_gps(nmea.true_course)

                    mav_gps_fix_type = nmea.get_mavlink_fix()
                    number_of_satellites = 5  # <--- TODO: FIXME

                    mavlink.send_raw_gps(nmea.ts_us, mav_gps_fix_type, nmea.lat, nmea.lon, nmea.alt, 
                        nmea.speed, number_of_satellites)
                    mavlink.send_gps(est_lat, est_lon, nmea.alt)

        elif oneInput == imu_sock:
            pkt, addr = get_last_packet(imu_sock, 128)
            try:
                imu.parse(pkt)
            except Exception as ee:
                print('failed to parse imu packet:', ee)
            else:
                rot = transforms3d.quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])

                if IMU_XFRM is not None:
                    rot = np.dot(rot, IMU_XFRM)

                pitch, roll, _yaw = transforms3d.euler.mat2euler(rot)

                calcHeading.update_gyro_yaw(-_yaw)
                #calcHeading.update_magnet_yaw(imu.magX, imu.magY, imu.magZ)

                est_heading = calcHeading.est_heading_degrees
                mavlink.send_attitude(roll, pitch, calcHeading.est_heading_radians)

                #shaft_pps = (imu.shaft_a_pps + imu.shaft_b_pps) * 0.5
                shaft_pps = imu.shaft_pps
                est_speed = shaft_pps * config.SHAFT_ENCODER_DISTANCE

                # Moab modes:
                # 0 - no signal from transmitter (radio timeout)
                # 1 - stop with brakes on
                # 2 - manual
                # 3 - auto_pilot
                # 4 - stop with throttle neutral

                # Map to mavlink modes:
                if imu.moab_mode == 0:
                    mavlink.custom_mode = 4 # stop
                    mavlink.heartbeat(force=True)
                elif imu.moab_mode == 1:
                    mavlink.custom_mode = 4 # stop
                    mavlink.heartbeat(force=True)
                elif imu.moab_mode == 2:
                    mavlink.custom_mode = 0 # manual
                    mavlink.heartbeat(force=True)
                elif imu.moab_mode == 3:
                    mavlink.custom_mode = 10 # auto
                    mavlink.heartbeat(force=True)
                elif imu.moab_mode == 4:
                    mavlink.custom_mode = 4 # stop
                    mavlink.heartbeat(force=True)



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
                mavlink.send_attitude(0, 0, hdg)
                mavlink.send_gps(est_lat, est_lon, 4000)

        elif oneInput == sbus_sock:
            pkt, addr = get_last_packet(sbus_sock, 64)
            try:
                flight_mode = sbus.parse_packet(pkt)
            except Exception as ee:
                print('failed to parse S.Bus packet:', ee)
            else:
                if flight_mode is not None:
                    if flight_mode == Flight_Mode.STOP:
                        mavlink.custom_mode = 4
                        mavlink.heartbeat(force=True)
                    elif flight_mode == Flight_Mode.MANUAL:
                        mavlink.custom_mode = 0
                        mavlink.heartbeat(force=True)
                    elif flight_mode == Flight_Mode.AUTO:
                        mavlink.custom_mode = 10
                        #mavlink.custom_mode = 15 # Guided
                        mavlink.heartbeat(force=True)
                    else:
                        print("BUG: unknown flight mode")

        elif oneInput == gcs_msgs_sock:
            pkt, addr = get_last_packet(gcs_msgs_sock, 100)
            try:
                mavlink.send_text_message(pkt)
            except Exception as ee:
                print('failed to mavlink.send_text_message():', ee)


    #shaft.update_speed_estimate()
    #print('speed: %.02f' % (est_speed))
    #sys.stdout.flush()
    mavlink.send_vfr_hud(est_speed)
    if est_lat is not None and est_lon is not None and est_heading is not None:
        nav_udp_packet = struct.pack('!dddd', est_lat, est_lon, est_heading, est_speed)
        for oneHost in config.Nav_Recipients:
            nav_sock.sendto(nav_udp_packet, oneHost)



