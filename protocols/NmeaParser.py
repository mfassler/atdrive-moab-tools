
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
    def __init__(self):
        self._avg_lag = 0.0
        self.ecefX = 0
        self.ecefY = 0
        self.ecefZ = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.speed = None
        self.true_course = None
        self.ts = None
        self.ts_us = None

        self.GGA_fix = 0
        # NMEA-GGA fix is this:
        #   0: none, 1: GPS, 2: DGPS, 4: RTK-fix, 5: RTK-float, 7: Manual, 8: Simulation
        self.GGA_numsats = 0

        self.GSA_fix = 0
        # NMEA-GSA fix is this:
        #   0: none, 1: no fix, 2: 2d fix, 3: 3d fix

        self.RMC_status = None
        # NMEA-RSA status is this:
        #   'A': active, 'V': void
        #  we will set to Python's True or None

        self.mavlink_fix = 0
        # Mavlink fix is this:
        #   0: no GPS connected
        #   1: GPS connected, but no fix
        #   2: 2D fix
        #   3: 3D fix
        #   4: DGPS / SBAS  (called "3D+DGPS" in apm-planner2)
        #   5: RTK float    (5 is "3D+RTK" in APM planner 2)
        #     ... anything higher than 5 is just "." in APM Planner2
        #   6: RTK fix
        #   7: static fix (base station)
        #   8: PPP, 3D position

    def get_mavlink_fix(self):
        if self.RMC_status:
            if self.GSA_fix == 2:  # 2-D fix
                return 2
            elif self.GSA_fix == 3:  # 3-D fix
                if self.GGA_fix == 1:  #plain GPS, no RTK
                    return 3
                elif self.GGA_fix == 2: # DGPS
                    return 4
                elif self.GGA_fix == 4: # RTK-fix
                    return 5  # APM_Planner2 will call this "3D+RTK"
                elif self.GGA_fix == 5:  # RTK-float
                    return 4  # APM_Planner2 will call this "3D+DGPS"
                else:  # hunh?
                    return 1
            else:
                return 1
        else:
            return 0


    def parse_packet(self, pkt):
        if pkt.startswith(b'$G'):
            if pkt[3:6] == b'RMC':
                self.parse_RMC(pkt)
            elif pkt[3:6] == b'GGA':
                self.parse_GGA(pkt)
            elif pkt[3:6] == b'GSA':
                self.parse_GSA(pkt)
            #elif pkt[3:6] == b'GLL':
            #    self.parse_GLL(pkt)


    def parse_GLL(self, pkt):
        '''Geographic Latitude and Longitude.  Obsolete, do not use.'''
        pass


    def parse_GGA(self, pkt):
        '''essential fix data'''

        #print('parse_GGA:', pkt)
        pieces = pkt.split(b',')
        # pieces[0] -> "$??GGA"
        # pieces[1] -> time-of-day timestamp
        # pieces[2] -> latitude
        # pieces[3] -> lat hemisphere, "N" or "S"
        # pieces[4] -> longitude
        # pieces[5] -> lon hemisphere, "E" or "W"
        # pieces[6] -> fix quality (0: none, 1: GPS, 2: DGPS, 4: RTK-fix, 5: RTK-float, 7: Manual, 8: Simulation)
        try:
            self.GGA_fix = int(pieces[6], 10)
        except:
            self.GGA_fix = None
        # pieces[7] -> number of satellites being tracked
        try:
            self.GGA_numsats = int(pieces[7], 10)
        except:
            self.GGA_numsats = 0
        # pieces[8] -> HDoP
        # pieces[9] -> altitude, above mean sea leveal
        # pieces[10] -> units for ALT-MSL (usually "M" for meters)
        # pieces[11] -> height of geoid (mean sea level) above WGS84 ellipsiod
        # pieces[12] -> units for geoid height (usually "M" for meters)
        # pieces[13] -> empty?   (seconds since last DGPS update?...)
        # pieces[14] -> empty?   (DGPS station ID number?...)


    def parse_GSA(self, pkt):
        '''DOP and active satellites'''

        #print('parse_GSA:', pkt)
        pieces = pkt.split(b',')
        # pieces[0] -> "$??GSA"
        # pieces[1] -> how to select fix "A" for auto, "M" for manual
        # pieces[2] -> fix type: 1: no fix, 2: 2D fix, 3: 3D fix
        try:
            self.GSA_fix = int(pieces[2], 10)
        except:
            self.GSA_fix = -1
        # pieces[3:-3] -> PRNs of satellites
        # pieces[-3] -> PDoP (Dilution of Precision)
        # pieces[-2] -> HDoP
        # pieces[-1] -> VDoP
        # literal '*'
        # checksum


    def parse_GSV(self, pkt):
        '''Satellites in view'''

        #print('parse_GSV:', pkt)
        pieces = pkt.split(b',')
        # pieces[0] -> "$??GSV"
        # pieces[1] -> number of sentences
        # pieces[2] -> current sentence number
        # pieces[3]
        ## 4 values per satellite:
        # pieces[4*i + 4] -> PRN number
        # pieces[4*i + 5] -> Elevation, degrees
        # pieces[4*i + 6] -> Azimuth, degrees
        # pieces[4*i + 7] -> SNR, higher is better, range 0 to 99
        # a literal '*'
        #  -> checksum


    def parse_RMC(self, pkt):
        '''Recommended Minimum'''

        #print('parse_RMC:', pkt)
        pieces = pkt.split(b',')
        # pieces[0] -> "$??RMC"
        # pieces[1] -> time-of-day timestamp
        # pieces[2] -> status:  "A" for active, or "V" for void
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
            self.RMC_status = True
        else:
            self.RMC_status = None
            return

        pyTime = datetime.datetime.strptime(
                pieces[1].decode() + ' ' + pieces[9].decode(), '%H%M%S.%f %d%m%y')
        pyTime = pyTime.replace(tzinfo=pytz.UTC)
        #print(pyTime.timestamp() - time.time())
        self.ts = pyTime.timestamp()

        self.ts_us = int(round(self.ts*1e6))
        self.lat = convertLatLon(pieces[3], pieces[4])
        self.lon = convertLatLon(pieces[5], pieces[6])
        try:
            self.speed = float(pieces[7]) * 0.5144444  # knots to m/s
        except:
            self.speed = None

        try:
            self.true_course = float(pieces[8])
        except:
            self.true_course = None


