

## From Moab:
#UBLOX_RX_PORT = 27110
NMEA_RX_PORT = 27113
IMU_RX_PORT = 27114


## Typical nav ports:
#NAV = 27201
#NAV = 27202
#  ...
#NAV = 27208
#NAV = 27209

## From auto-pilot speed control:
PID_CONTROL_PORT = 27311  # (debug info only)


## From Moab:
MOAB_STATUS_PORT = 31337  # (debug info only)
SBUS_PORT = 31338

## Msgs that will be sent to Mavlink GCS:
GCS_MESSAGES_PORT = 31339

## From Moab:
RADIO169_PORT = 31340
BUTTON_PORT = 31345

## From the Lidar software:
FOLLOW_AVOID_PORT = 52537
#LIDAR_NAV_RX_PORT = 11546


from .FollowAvoid import FollowAvoid
from .ImuPacket import ImuPacket
from .NmeaParser import NmeaParser

from .Radio169 import Radio169
from .SbusParser import SbusParser

from .PidParser import PidParser
