
import struct


'''
this is the packet structure in C:

    struct multi_data {

        // 64 bits:
        int16_t compass_XYZ[3];  // external compass
        int16_t _padding1;  // the compiler seems to like 64-bit boundaries

        // 3 * 64 bits:
        char bnoData[22];  // internal IMU
        int16_t _padding2;  // the compiler seems to like 64-bit boundaries

        // 64 bits:
        float temperature; // degrees celsius, no need for high accuracy

        // Pressure:  typical sensor value is ~100000, with accuracy of +/- 12.0,
        // (don't forget to convert between Pa and hPa), so this is well
        // within the accuracy of float32
        float pressure;

        // 64 bits:
        uint16_t sbus_a;
        uint16_t sbus_b;
        uint16_t _padding3;
        uint16_t _padding4;

        // 64 bits:
        // TODO:  do we really need float64 for these numbers?
        double shaft_pps;

    } mData;


'''

class ImuPacket:
    def __init__(self):
        pass

    def parse(self, pkt):
        self.magX, self.magY, self.magZ, self._padding1, \
            self.qw, self.qx, self.qy, self.qz, \
            self.lax, self.lay, self.laz, self.gx, self.gy, self.gz, \
            self.imu_temp, self.calib_stat, \
            self._padding2, \
            self.temperature, self.pressure, \
            self.sbus_a, self.sbus_b, self._padding3, self._padding4, \
            self.shaft_pps = struct.unpack('<hhhhhhhhhhhhhhbBhffHHhhd', pkt)



