
import sys
import math
import time
import socket
import struct
import numpy as np

from builtins import object
from pymavlink.dialects.v20 import ardupilotmega as mavlink1



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
        self._mav = mavlink1.MAVLink(f, srcSystem=1, srcComponent=1)

        self._last_heartbeat_time = time.time()
        self._last_attitude_time = time.time()
        self._last_gps_time = time.time()
        self._last_vfr_hud_time = time.time()
        self._heading = 0
        self._mission_count = 0
        self._mission_items = []
        self._mission_items_int = []
        self.custom_mode = 0

    def heartbeat(self, force=False):
        if force or ((time.time() - self._last_heartbeat_time) > 1.0):
            self._last_heartbeat_time = time.time()
            ## _type, autopilot, base_mode, custom_mode, system_status, mavlink_version
            _type = 10
            autopilot = 3
            #base_mode = 0xff # 128 | 64 | 8 | 4
            base_mode = 128 | 64 | 16 | 8 | 4
            #custom_mode = 65
            #custom_mode = 0 # Manual
            #custom_mode = 4 # Hold
            custom_mode = 10 # Auto
            #custom_mode = 15 # Guided
            system_status = 4
            mavlink_version = 3

            # Manual Mode:
            #_type = 10
            #autopilot = 3
            #base_mode = 193
            #custom_mode = 4
            #system_status = 5
            #mavlink_version = 3

            # HOLD Mode:
            #_type = 10
            #autopilot = 3
            #base_mode = 192 # 129
            #custom_mode = 4
            #system_status = 5
            #mavlink_version = 3

            msg = mavlink1.MAVLink_heartbeat_message(
                _type, autopilot, base_mode, self.custom_mode, system_status, mavlink_version)

            msgBuf = msg.pack(self._mav)
            self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

    def send_attitude(self, roll, pitch, yaw):
        "roll, pitch, and yaw are in radians"
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
                roll, pitch, yaw,
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

    def send_raw_gps(self, ts, gpsFix, lat, lon, alt, vel_ms, num_satellites):
        ts = 0  #int(round(ts * 1e6))
        lat = int(round(lat * 1e7))
        lon = int(round(lon * 1e7))
        alt = int(round(alt * 1000))
        eph, epv, cog = 65535, 65535, 65535
        try:
            vel = int(round(vel_ms / 100.0))
        except:
            vel = 65535

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

    def send_text_message(self, message):
        msg = mavlink1.MAVLink_statustext_message(0, message)
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
            self._mission_items_int = [None] * self._mission_count
            for i in range(self._mission_count):
                self.mission_request(i)
        elif oneMsg.name == 'MISSION_ITEM':
            print(oneMsg.name)
            #self.send_text_message(b"hi there")
            #return
            item = oneMsg.to_dict()
            self._a_mission_item = item
            if item['param2'] < 0.01:
                print("Guided Mode")
                return
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
                f.flush()
                sys.stdout.flush()
                self.send_text_message(b"Wrote Mission")
        elif oneMsg.name == 'MISSION_ITEM_INT':
            print(oneMsg.name)
            item = oneMsg.to_dict()
            self._a_mission_item = item
            #if item['param2'] < 0.01:
            #    print("Guided Mode")
            #    return
            self._mission_items_int[item['seq']] = item
            allSomething = True
            for oneItem in self._mission_items_int:
                if oneItem is None:
                    allSomething = False
                    #print("missing a waypoint!")
            if allSomething:
                print("received complete mission")
                f = open('CURRENT_MISSION_int.txt', 'wb')
                f.write(b'QGC WPL 110\n')
                for oneItem in self._mission_items_int:
                    outStr1 = b"%d\t%d\t%d\t%d\t%g\t%g\t%g\t%g\t" % (oneItem['seq'], oneItem['current'],
                        oneItem['frame'], oneItem['command'], oneItem['param1'], oneItem['param2'],
                        oneItem['param3'], oneItem['param4'])
                    outStr2 = b"%.7f\t%.7f\t%g\t%d" % (oneItem['x'] / 10000000.0, oneItem['y'] / 10000000.0, oneItem['z'], oneItem['autocontinue'])
                    f.write(outStr1 + outStr2 + b'\n')
                f.close()

        elif oneMsg.name == 'PARAM_REQUEST_LIST':
            print(oneMsg.name)
            PARAMS = [
                (b'MODE_CH', 5, 2),
                (b'MODE3', 4.0, 2),
                (b'MODE2', 11.0, 2),
                (b'INITIAL_MODE', 0.0, 2),
                (b'myStuff', 11, 2),
            ]
            #total = 664 # len(PARAMS)
            total = len(PARAMS)
            for i, item in enumerate(PARAMS):
                # param_id, param_value, param_type, param_count, param_index
                msg = mavlink1.MAVLink_param_value_message(item[0], item[1], item[2], total, i)
                msgBuf = msg.pack(self._mav)
                self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))
            #msgBuf = b'\xfe\x19\x87\x01\x01\x16\x00\x00\x80@\x98\x02\x1d\x00MODE1\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02d"'
            #self._sock.sendto(msgBuf, (self._remote_addr, self._remote_port))

        elif oneMsg.name == 'PARAM_REQUEST_READ':
            print(oneMsg.name)
            self._aMsg = oneMsg
        elif oneMsg.name == 'SET_MODE':
            print(oneMsg.name)
            if oneMsg.custom_mode == 0:
                print('  ### MANUAL')
            elif oneMsg.custom_mode == 4:
                print('  ### HOLD')
            elif oneMsg.custom_mode == 10:
                print('  ### AUTO')
                if self.custom_mode == 10 or self.custom_mode == 15:
                    self.custom_mode = 10
                    self.heartbeat(force=True)
            elif oneMsg.custom_mode == 15:
                print('  ### GUIDED')
                if self.custom_mode == 10 or self.custom_mode == 15:
                    self.custom_mode = 15
                    self.heartbeat(force=True)
            else:
                print("uknown mode:", oneMsg.custom_mode)
            
        else:
            print(oneMsg.name)
            self._whee = oneMsg
            print(self._whee.to_json())


