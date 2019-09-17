

import struct
import time
import datetime
import pytz





def convertLatLon(inStr, hemisphere):
    pieces = inStr.strip().split(b'.')
    minutesStr = pieces[0][:-2]
    secondsStr = pieces[0][-2:] + b'.' + pieces[1]
    minutes = float(minutesStr)
    seconds = float(secondsStr)
    outFloat = minutes + seconds / 60.0
    if hemisphere == b'N':
        pass
    elif hemisphere == b'S':
        outFloat *= -1
    elif hemisphere == b'E':
        pass
    elif hemisphere == b'W':
        outFloat *= -1
    else:
        print('unknown hemisphere', hemisphere)
    return outFloat





class NmeaParser:
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


    def parse_nmea_packet(self, pkt):
        if pkt.startswith(b'$GPRMC'):
            self.parse_RMC(pkt)
        elif pkt.startswith(b'$GNRMC'):
            self.parse_RMC(pkt)
        elif pkt.startswith(b'$GNGLL'):
            self.parse_GLL(pkt)

    def parse_GLL(self, pkt):
        #print('parsing GLL')
        pieces = pkt.split(b',')
        self.gpsFix = 3
        ts = time.time()
        ts_us = int(round(ts*1e6))
        self.lat = convertLatLon(pieces[1], pieces[2])
        self.lon = convertLatLon(pieces[3], pieces[4])
        self._mavlink.send_raw_gps(ts_us, self.gpsFix, self.lat, self.lon, self.alt, 5)

    def parse_RMC(self, pkt):
        print('parsing RMC...')
        pieces = pkt.split(b',')
        ## From the NMEA protocol:
        # pieces[0] -> "$GPRMC"
        # pieces[1] -> time-of-day timestamp
        # pieces[2] -> "A" for okay, or "V" for warning
        # pieces[3] -> latitude
        # pieces[4] -> lat hemisphere, "N" or "S"
        # pieces[5] -> longitude
        # pieces[6] -> lon hemisphere, "E" or "W"
        # pieces[7] -> speed
        # pieces[8] -> true course
        # pieces[9] -> date, eg:  "140418" for 14-April-2018
        # pieces[10] -> variation
        # pieces[11] -> east/west
        # pieces[12] -> checksum
        if pieces[2] == b'A':
            self.gpsFix = 3
        else:
            self.gpsFix = 0
            return

        pyTime = datetime.datetime.strptime(
                pieces[1].decode() + ' ' + pieces[9].decode(), '%H%M%S.%f %d%m%y')
        pyTime = pyTime.replace(tzinfo=pytz.UTC)
        #print(pyTime.timestamp() - time.time())
        ts = pyTime.timestamp()
        #print(ts)
        ts_us = int(round(ts*1e6))
        self.lat = convertLatLon(pieces[3], pieces[4])
        self.lon = convertLatLon(pieces[5], pieces[6])
        self._mavlink.send_raw_gps(ts_us, self.gpsFix, self.lat, self.lon, self.alt, 5)



