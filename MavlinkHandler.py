
import sys
import math
import time
import socket
import struct
import numpy as np

from builtins import object
from pymavlink.dialects.v10 import ardupilotmega as mavlink1



class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


class MavlinkHandler:
    def __init__(self, remote_addr, remote_port = 14550):
        self._BOOTUP_TIME = time.time()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._remote_addr = remote_addr
        self._remote_port = remote_port
        self._sock.bind(("0.0.0.0", 0))
        f = fifo()
        self._mav = mavlink1.MAVLink(f)

        self._last_heartbeat_time = time.time()
        self._last_attitude_time = time.time()
        self._last_gps_time = time.time()
        self._last_vfr_hud_time = time.time()
        self._heading = 0
        self._mission_count = 0
        self._mission_items = []

    def heartbeat(self):
        if (time.time() - self._last_heartbeat_time) > 1.0:
            self._last_heartbeat_time = time.time()
            ## _type, autopilot, base_mode, custom_mode, system_status, mavlink_version
            _type = 10
            autopilot = 0
            base_mode = 128 + 64
            custom_mode = 65
            system_status = 4
            mavlink_version = 1
            msg = mavlink1.MAVLink_heartbeat_message(
                _type, autopilot, base_mode, custom_mode, system_status, mavlink_version)

            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    def send_attitude(self, roll, pitch, yaw):
        self.heartbeat()
        self._heading = int(round(yaw*100))
        while self._heading < 0:
            self._heading += 36000
        if (time.time() - self._last_attitude_time) > 0.1:
            self._last_attitude_time = time.time()
            time_boot_ms = int( (time.time() - self._BOOTUP_TIME) * 1000)
            rollspeed, pitchspeed, yawspeed = 0,0,0

            msg = mavlink1.MAVLink_attitude_message(
                time_boot_ms,
                math.radians(roll), math.radians(pitch), math.radians(yaw),
                rollspeed, pitchspeed, yawspeed
            )
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    def send_gps(self, _lat, _lon, _alt):
        self.heartbeat()
        if (time.time() - self._last_gps_time) > 0.1:
            self._last_gps_time = time.time()
            time_boot_ms = int( (time.time() - self._BOOTUP_TIME) * 1000)

            lat = int(round(_lat * 1e7))
            lon = int(round(_lon * 1e7))
            alt = int(round(_alt * 1000))
            relative_alt = alt
            vx, vy, vz, hdg = 0,0,0, self._heading
            msg = mavlink1.MAVLink_global_position_int_message(
                time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
            )
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    def send_raw_gps(self, ts, gpsFix, lat, lon, alt, num_satellites):
        ts = 0  #int(round(ts * 1e6))
        lat = int(round(lat * 1e7))
        lon = int(round(lon * 1e7))
        alt = int(round(alt * 1000))
        eph, epv, vel, cog = 65535, 65535, 65535, 65535
        msg = mavlink1.MAVLink_gps_raw_int_message(
            ts, gpsFix, lat, lon, alt, eph, epv, vel, cog, num_satellites
        )
        msgBuf = msg.pack(self._mav)
        self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    #def send_vfr_hud(self, airspeed, groundspeed, heading, throttle, alt, climb):
    def send_vfr_hud(self, groundspeed):
        self.heartbeat()
        if (time.time() - self._last_vfr_hud_time) > 0.1:
            self._last_vfr_hud_time = time.time()
            hdg = int(round(self._heading / 100))
            msg = mavlink1.MAVLink_vfr_hud_message(groundspeed, groundspeed, hdg, 0, 0, 0)
            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    def mission_request_list(self):
        msg = mavlink1.MAVLink_mission_request_list_message(1, 190)
        msgBuf = msg.pack(self._mav)
        print('lalala')
        self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))


    def mission_request(self, seq):
        msg = mavlink1.MAVLink_mission_request_message(0, 0, seq)
        msgBuf = msg.pack(self._mav)
        print('... mission request:', seq)
        self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))


    def do_message(self, oneMsg):
        if oneMsg.name == 'MISSION_COUNT':
            the_msg = oneMsg.to_dict()
            self._mission_count = the_msg['count']
            #mavlink.mission_request_list()
            print("\n ---- %s: %d" % (oneMsg.name, self._mission_count))
            self._mission_items = [None] * self._mission_count
            for i in range(self._mission_count):
                self.mission_request(i)
        elif oneMsg.name == 'MISSION_ITEM':
            print(oneMsg.name)
            item = oneMsg.to_dict()
            self._mission_items[item['seq']] = item
            allSomething = True
            for oneItem in self._mission_items:
                if oneItem is None:
                    allSomething = False
                    #print("missing a waypoint!")
            if allSomething:
                f = open('/home/fassler/CURRENT_MISSION.txt', 'wb')
                print("\n\n  THE MISSION:")
                for i in range(len(self._mission_items)):
                    print(self._mission_items[i]['x'], self._mission_items[i]['y'])
                    f.write(b"%f, %f\n" % (self._mission_items[i]['x'], self._mission_items[i]['y']))
                print("\n")
                f.close()
                sys.stdout.flush()



