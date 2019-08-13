

import struct
import time
import datetime

from _3rd_party.pymap3d.ecef2geodetic import ecef2geodetic


_GPS_EPOCH = datetime.datetime(1980, 1, 6, 0,0,0,0, datetime.timezone.utc)
_GPS_EPOCH_TS = _GPS_EPOCH.timestamp()
_GPS_LEAP_SECONDS = 18.0
_AVG_LAG = 0.08


def convert_gpstime_to_python_time(iTOW, fTOW, week):
    ts = _GPS_EPOCH_TS + week*7*24*60*60 + iTOW/1000 + fTOW/1e9 - _GPS_LEAP_SECONDS  + _AVG_LAG
    return ts


def fletcher_checksum(_byteArray):
    ckA = 0
    ckB = 0
    for i, oneByte in enumerate(_byteArray):
        ckA = ckA + oneByte
        ckB = ckB + ckA
    return (ckA & 0xff) | ((ckB & 0xff) << 8)



class_names = [
    None,  'NAV', 'RXM', None, 'INF', 'ACK', 'CFG', None, None, 'UPD',
    'MON', 'AID', None, 'TIM'
]


## Python struct:
# b int8_t    I1
# B uint8_t   U1
# h int16_t   I2
# H uint16_t  U2
# i int32_t   I4
# I uint32_t  U4



class UbloxParser:
    def __init__(self, mavlink):
        self._avg_lag = 0.0
        self.ecefX = 0
        self.ecefY = 0
        self.ecefZ = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.gpsFix = 0
        self._mavlink = mavlink


    def parse_ublox_packet(self, pkt):
        if len(pkt) < 8:
            print("UDP packet is too small")
            return

        sync1, sync2, msgClass, msgId, payload_length = struct.unpack('<BBBBH', pkt[:6])

        if sync1 != 0xb5 or sync2 != 0x62:
            print("Not a ublox packet!")
            return

        payload = pkt[6:-2]
        if payload_length != len(payload):
            print("Wrong payload size!")
            return

        check_range = pkt[2:-2]
        cksum, = struct.unpack('<H', pkt[-2:])
        if cksum != fletcher_checksum(check_range):
            print("Incorrect checksum!")
            return

        self.parse_ublox_payload(msgClass, msgId, payload)

    def parse_ublox_payload(self, msgClass, msgId, payload):
        if msgClass == 1:
            self.parse_nav_packet(msgId, payload)

        else:
            if msgClass < len(class_names) - 1:
                classText = class_names[msgClass]
            else:
                classText = str(msgClass)
            print(classText, msgId, len(payload))


    def parse_nav_packet(self, msgId, payload):
        if msgId == 4:
            self.parse_NAV_DOP(payload)
        elif msgId == 6:
            self.parse_NAV_SOL(payload)
        elif msgId == 7:
            self.parse_NAV_PVT(payload)
        else:
            print("in parse_mav_packet(), unknown msgId:", msgId)


    def parse_NAV_DOP(self, payload):
        assert len(payload) == 18
        iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP = struct.unpack(
            '<IHHHHHHH', payload)
        #print("vDOP: %d, hDOP: %d" % (vDOP, hDOP))

    def parse_NAV_SOL(self, payload):
        '''
        iTOW - ms - GPS time of week
        fTOW - ns - fractional part of iTOW
        week - GPS week number
        gpsFix - enum - 0:none, 1:deadReckoning, 2:2dfix, 3:3dfix, 4:gps+deadReckoning, 5:time-only
        ecefX, ecefY, ecefZ - cm - ECEF coords
        '''
        global _AVG_LAG
        assert len(payload) == 52
        iTOW, fTOW, week, gpsFix, flags, ecefX, ecefY, ecefZ, pACC, \
        ecefVX, ecefVY, ecefVZ, sAcc, pDOP, res1, numSV, res2 = struct.unpack(
            '<IihBBiiiIiiiIHBB4s', payload)

        lat, lon, alt = ecef2geodetic(ecefX/100.0, ecefY/100.0, ecefZ/100.0)

        self.gpsFix = gpsFix
        self.ts = convert_gpstime_to_python_time(iTOW, fTOW, week)
        ts = time.time()
        #print(iTOW, fTOW, week, gpsFix, ecefX, ecefY, ecefZ, lat, lon, alt)
        if gpsFix == 3:
            self.ecefX = ecefX
            self.ecefY = ecefY
            self.ecefZ = ecefZ
            self.lat = lat
            self.lon = lon
            self.alt = alt
            #lag = ts - gps_ts
            #self._avg_lag = 0.9 * self._avg_lag + 0.1 * lag

        ts_us = int(round(self.ts * 1000000))
        # TODO:  I think the mavlink definition of gpsFix is different
        # than the UBlox definition?
        self._mavlink.send_raw_gps(ts_us, gpsFix, lat, lon, alt, numSV)

    def parse_NAV_PVT(self, payload):
        assert len(payload) == 92



