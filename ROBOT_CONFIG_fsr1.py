

MOAB_IP_ADDRESS = "192.168.31.201"
MAVLINK_IP_ADDRESS = "192.168.31.80"

Nav_Recipients = [
    ('127.0.0.1', 27201),
    ('127.0.0.1', 27202),
]

SHAFT_ENCODER_DISTANCE = 0.045  # 4.5 cm per tick

compass_x_range = 2142.0
compass_y_range = 2051.5
compass_z_range = 2090.0 # not used

compass_x_center = 562.0
compass_y_center = -1216.5
compass_z_center = 0.0 # not used

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

