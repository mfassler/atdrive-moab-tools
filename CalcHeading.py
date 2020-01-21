
import numpy as np


class CalcHeading:
    def __init__(self):
        self._gyro_offset_vec = np.array([1.0, 0.0])  # x,y of the offset angle
        self._offset_radians_avg = 0.0
        self.est_heading_radians = 0.0
        self.est_heading_degrees = 0.0

    def calibrate_from_gps(self, nmea_true_course):
        if nmea_true_course is None:
            return
        assert isinstance(nmea_true_course, (int, float))

        offset_radians = np.radians(nmea_true_course) - self._gyro_yaw
        #offset_radians = np.radians(nmea_true_course) - self._magnet_yaw
        #offset_angle = 0.0
        _offset_x = np.cos(offset_radians)
        _offset_y = np.sin(offset_radians)
        self._gyro_offset_vec = 0.99 * self._gyro_offset_vec + 0.01 * np.array([_offset_x, _offset_y])

        # Normalize to length 1:
        self._gyro_offset_vec /= np.linalg.norm(self._gyro_offset_vec)
        self._offset_radians_avg = np.arctan2(self._gyro_offset_vec[1], self._gyro_offset_vec[0])
        #print("offset: %.02f" % np.degrees(self._offset_radians_avg))

    def update_gyro_yaw(self, _yaw):
        self._gyro_yaw = _yaw
        while self._gyro_yaw < 0.0:
            self._gyro_yaw += 2 * np.pi

        self.est_heading_radians = self._gyro_yaw + self._offset_radians_avg
        self.est_heading_degrees = np.degrees(self.est_heading_radians)

    def update_magnet_yaw(self, x, y, z):
        print("Not implemented.")

