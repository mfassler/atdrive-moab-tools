

MOAB_IP_ADDRESS = "192.168.32.201"
MAVLINK_IP_ADDRESS = "192.168.32.80"

Nav_Recipients = [
    ('127.0.0.1', 27201),
    ('127.0.0.1', 27202),
]

#SHAFT_ENCODER_DISTANCE = 0.045  # 4.5 cm per tick
SHAFT_ENCODER_DISTANCE = 0.155  # 15.5 cm per tick

compass_x_range = 2200.0
compass_y_range = 2065.0
compass_z_range = 2100 # not used

compass_x_center = 450.0
compass_y_center = -640.0
compass_z_center = 0.0 # not used

## With Hammer-throw bracket:
#compass_x_range = 2178.0
#compass_y_range = 2042.0
#compass_z_range = 2100 # not used

#compass_x_center = 597.0
#compass_y_center = -204.5
#compass_z_center = 0.0 # not used



mag_declination = -7.57  # Tokyo

lidar_ref_angle = 45.0
lidar_ref_lat = 35.6535
lidar_ref_lon = 139.837


# The Moab can be mounted with any orientation, but you will
# have to provide the rotation matrix:

# Moab is mounted with "forward" pointing "left"
#   (ie:  ethernet cable is pointing "right")
IMU_XFRM = [
    [ 0, 1, 0],
    [-1, 0, 0],
    [ 0, 0, 1]
]

